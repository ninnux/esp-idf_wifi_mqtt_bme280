#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/i2c.h"

#include "esp_sleep.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"


#include "bme280.h"

#include "ninux_esp32_ota.h"

#include "ninux_sensordata_pb.h"

//#define SDA_PIN GPIO_NUM_21
//#define SCL_PIN GPIO_NUM_22

#define SDA_PIN GPIO_NUM_16
#define SCL_PIN GPIO_NUM_17

#define TAG_BME280 "BME280"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

//static const char *TAG = "MQTT_EXAMPLE";
//const char *TAG = "MQTT_EXAMPLE";

//static EventGroupHandle_t wifi_event_group;
EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

#define TIMESLOT 4 

static RTC_DATA_ATTR struct timeval sleep_enter_time;

RTC_DATA_ATTR char rtc_buffer[4000];
RTC_DATA_ATTR int rtc_buffer_len=0;

RTC_DATA_ATTR int deepsleep=0;
RTC_DATA_ATTR int counter=0;

uint8_t msgData[32];

SemaphoreHandle_t xSemaphore = NULL;

void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
  	    //sensordata_init2((unsigned char **) &rtc_buffer, &rtc_buffer_len);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    //const int ext_wakeup_pin_1 = 25;
    //const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    //const int ext_wakeup_pin_2 = 26;
    //const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    //printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    //esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    deepsleep=1;
    esp_deep_sleep_start();
}

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}

void task_bme280_normal_mode(void *ignore)
{
   struct bme280_t bme280 = {
   	.bus_write = BME280_I2C_bus_write,
   	.bus_read = BME280_I2C_bus_read,
   	.dev_addr = CONFIG_BME280_ADDRESS,
   	.delay_msec = BME280_delay_msek
   };
   int i=0;
   int h=0;
   int t=0;
   int p=0;
   float hsum=0;
   float tsum=0;
   float psum=0;

   s32 com_rslt;
   s32 v_uncomp_pressure_s32;
   s32 v_uncomp_temperature_s32;
   s32 v_uncomp_humidity_s32;
   if( xSemaphore != NULL )
   {
       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
       {

	com_rslt = bme280_init(&bme280);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	if (com_rslt == SUCCESS) {
	 i=0;
	 for(i=0;i<10;i++){
	   vTaskDelay(40/portTICK_PERIOD_MS);
	   com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
	   	&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);
	   if (com_rslt == SUCCESS) {
	     hsum=bme280_compensate_humidity_double(v_uncomp_humidity_s32);
	     psum=bme280_compensate_pressure_double(v_uncomp_pressure_s32);
	     tsum=bme280_compensate_temperature_double(v_uncomp_temperature_s32);
	   } else {
	     ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
	   }
	 }
	 hsum=0;
         psum=0;
         tsum=0;
	 for(i=0;i<10;i++){
	   vTaskDelay(40/portTICK_PERIOD_MS);
	   com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
	   	&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

	   if (com_rslt == SUCCESS) {
	     hsum+=bme280_compensate_humidity_double(v_uncomp_humidity_s32);
	     psum+=bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
	     tsum+=bme280_compensate_temperature_double(v_uncomp_temperature_s32);
	     //ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
	     //bme280_compensate_temperature_double(v_uncomp_temperature_s32),
	     //bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
	     //bme280_compensate_humidity_double(v_uncomp_humidity_s32));
	   } else {
	     ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
	   }
	 }
	 h=hsum/i*10;
	 p=psum/i*10;
	 t=tsum/i*10;
	 printf("hum:%d,temp:%d,pres:%d\n",h,t,p);
	 //sprintf((char*)msgData,"{\"hum\":%d,\"temp\":%d,\"pres\":%d}",h,t,p);
	 //printf("%s",msgData);
  	 char* keys[]={"pres","temp","hum"}; 
  	 int values[]={p,t,h};
  	 sensordata_insert_values2((unsigned char **) &rtc_buffer,counter,keys,values,3,&rtc_buffer_len);
	
	} else {
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}

	xSemaphoreGive( xSemaphore );
       }
    
   }
	
   	if(counter%TIMESLOT!=0){
		counter+=1;
		sleeppa(10);
        };
	vTaskDelete(NULL);
}


static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char mqtt_topic[128];
    bzero(mqtt_topic,sizeof(mqtt_topic));
    sprintf(mqtt_topic,"ambiente/%s/ninuxsensordata_pb",CONFIG_MQTT_NODE_NAME);
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, mqtt_topic,rtc_buffer, rtc_buffer_len, 1, 0);
            //msg_id = esp_mqtt_client_publish(client, mqtt_topic,(const char *) msgData, 0, 1, 0);
	 
	    rtc_buffer_len=0;
  	    sensordata_init2((unsigned char **) &rtc_buffer, &rtc_buffer_len);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      	    //xSemaphoreGive( xSemaphore );
      	    vTaskDelete(NULL); 
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static int s_retry_num = 0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < 3) {
            	esp_wifi_connect();
            	xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
                s_retry_num++;
	    }else{
		printf("restart!!\n");
		esp_restart();	
	    } 
            break;
	}
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    //esp_wifi_set_max_tx_power(40);
    //esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };


   while (1) {
   	if( xSemaphore != NULL )
   	{


#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */


    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) { 
      printf("semaforo libero");
      esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
      esp_mqtt_client_start(client);
      
      //vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
      counter+=1;
      sleeppa(10);
      



    }else{
      //printf("semaforo occupato");
    }
    }
  }
}

void app_main()
{
    printf("counter:%d\n",counter);
    printf("deepsleep:%d\n",deepsleep);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);




    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
   if(counter%TIMESLOT==0 || deepsleep==0){
    wifi_init();
    esp_ota_mark_app_valid_cancel_rollback(); 
    ninux_esp32_ota();
   }


    vSemaphoreCreateBinary( xSemaphore );
    vTaskDelay( 1000 / portTICK_RATE_MS );
    //xTaskCreate( &DHT_task, "DHT_task", 2048, NULL, 5, NULL );
    i2c_master_init();
    xTaskCreate(&task_bme280_normal_mode, "bme280_normal_mode",  2048, NULL, 6, NULL);
    vTaskDelay( 3000 / portTICK_RATE_MS );

   if(counter%TIMESLOT==0 || deepsleep==0){
   	 mqtt_app_start();
   }else{
      counter+=1;
   } 
   
	 
}
