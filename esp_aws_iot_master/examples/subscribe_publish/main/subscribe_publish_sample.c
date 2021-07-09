/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file subscribe_publish_sample.c
 * @brief simple MQTT publish and subscribe on the same topic
 *
 * This example takes the parameters from the build configuration and establishes a connection to the AWS IoT MQTT Platform.
 * It subscribes and publishes to the same topic - "test_topic/esp32"
 *
 * Some setup is required. See example README for details.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "subpub";

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;


/* CA Root certificate, device ("Thing") certificate and device
 * ("Thing") key.

   Example can be configured one of two ways:

   "Embedded Certs" are loaded from files in "certs/" and embedded into the app binary.

   "Filesystem Certs" are loaded from the filesystem (SD card, etc.)

   See example README for more details.
*/
#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)

static const char * DEVICE_CERTIFICATE_PATH = CONFIG_EXAMPLE_CERTIFICATE_PATH;
static const char * DEVICE_PRIVATE_KEY_PATH = CONFIG_EXAMPLE_PRIVATE_KEY_PATH;
static const char * ROOT_CA_PATH = CONFIG_EXAMPLE_ROOT_CA_PATH;

#else
#error "Invalid method for loading certs"
#endif

#define STORAGE_NAMESPACE "storage"

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;

void mLED_Power(bool sta);
void mLED_Ble(uint8_t sta);
void mLED_Wifi(uint8_t sta);
esp_err_t Read_User_Config(void);
esp_err_t Save_User_SSID_Password(void);

static uint8_t Ble_Led_Status = 0;
static uint8_t Wifi_Led_Status = 0;
static uint8_t Wifi_lost_cnts = 0;
static uint8_t App_Cmd;
static uint8_t Ble_Cmd_index = 0;

static bool b_wifi_is_connected = false;

typedef struct{
    uint8_t SSID[32];
    uint8_t Password[64]
}User_Config_t;

User_Config_t   u_config = {
    .SSID = {0},
    .Password = {0},
};

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        mLED_Wifi(1);
        printf("Wifi connected\n");
        b_wifi_is_connected = true;
        Wifi_lost_cnts = 0;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        if(Wifi_Led_Status == 2) //配网状态的闪烁
        {
            if(++Wifi_lost_cnts >= 15) //约30秒超时
            {
                mLED_Wifi(0);
            }
        }
        else
            mLED_Wifi(0);
        
        b_wifi_is_connected = false;
        printf("Wifi disconnect\n");
        break;
    default:
        break;
    }
    return ESP_OK;
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

void aws_iot_task(void *param) {
    char cPayload[100];

    int32_t i = 0;

    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    IoT_Publish_Message_Params paramsQOS0;
    IoT_Publish_Message_Params paramsQOS1;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
    mqttInitParams.pRootCALocation = ROOT_CA_PATH;
    mqttInitParams.pDeviceCertLocation = DEVICE_CERTIFICATE_PATH;
    mqttInitParams.pDevicePrivateKeyLocation = DEVICE_PRIVATE_KEY_PATH;
#endif

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

#ifdef CONFIG_EXAMPLE_SDCARD_CERTS
    ESP_LOGI(TAG, "Mounting SD card...");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
        abort();
    }
#endif

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    const char *TOPIC = "test_topic/esp32_Domyos_Dongle_Test";
    const int TOPIC_LEN = strlen(TOPIC);

    ESP_LOGI(TAG, "Subscribing...");
    rc = aws_iot_mqtt_subscribe(&client, TOPIC, TOPIC_LEN, QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }

    sprintf(cPayload, "%s : %d ", "hello from SDK", i);

    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *) cPayload;
    paramsQOS0.isRetained = 0;

    paramsQOS1.qos = QOS1;
    paramsQOS1.payload = (void *) cPayload;
    paramsQOS1.isRetained = 0;

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1000 / portTICK_RATE_MS);
        sprintf(cPayload, "%s : %d ", "hello from ESP32 Domyos Dongle (QOS0)", i++);
        paramsQOS0.payloadLen = strlen(cPayload);
        rc = aws_iot_mqtt_publish(&client, TOPIC, TOPIC_LEN, &paramsQOS0);

        sprintf(cPayload, "%s : %d ", "hello from ESP32 Domyos Dongle (QOS1)", i++);
        paramsQOS1.payloadLen = strlen(cPayload);
        rc = aws_iot_mqtt_publish(&client, TOPIC, TOPIC_LEN, &paramsQOS1);
        if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            ESP_LOGW(TAG, "QOS1 publish ack not received.");
            rc = SUCCESS;
        }
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    
    wifi_config_t wifi_config = {
        .sta = {
        .ssid = "123456",
        .password = "123456",
        },
    };

    if(Read_User_Config() == ESP_OK)
    {
        if((u_config.SSID[0] != 0) && (u_config.Password[0] != 0))
        {
            memcpy(wifi_config.sta.ssid, u_config.SSID, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, u_config.Password, sizeof(wifi_config.sta.password));
            mLED_Wifi(2);
        }
        else
            mLED_Wifi(0);
    }
    else
    {
        printf("No config, wifi is not init\n");
        mLED_Wifi(0);
    }

    ESP_LOGI(TAG, "Setting WiFi configuration SSID: %s...", wifi_config.sta.ssid);
    ESP_LOGI(TAG, "Password: %s...", wifi_config.sta.password);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}


static const int RX_BUF_SIZE = 1024;

/*
Uart1 - Console
    TXD1-IO10
    RXD1-IO9
Uart2 - BLE Model
    TXD2-IO15
    RXD2-IO2
*/
#define TXD1_PIN (GPIO_NUM_10)
#define RXD1_PIN (GPIO_NUM_9)

#define TXD2_PIN (GPIO_NUM_15)
#define RXD2_PIN (GPIO_NUM_2)

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/*int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}*/

int u1_sendData(unsigned char* data, unsigned short len)
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int u2_sendData(unsigned char* data, unsigned short len)
{
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

/*Receive from Console*/
static void rx1_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX1_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            u2_sendData(data, rxBytes);

            if(data[0] == 0xF0)
            {
                if((Ble_Led_Status == 0) && (Ble_Cmd_index == 1))
                {
                    if(data[1] == (App_Cmd + 0x10))
                    {
                        mLED_Ble(2);
                    }
                }
            }
        }
    }
    free(data);
}

/*Receive from BLE*/
static void rx2_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX2_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    char* p1;
    char* p2;
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 20 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            u1_sendData(data, rxBytes);

            if(data[0] == 0xF0)
            {
                if(Ble_Led_Status == 0)
                {
                    App_Cmd = data[1];
                    Ble_Cmd_index = 1;
                }
                else
                {
                    if(data[1] == 0x60)
                    {
                        if((data[2] & 0x01)==0x01)
                        {
                            mLED_Ble(1);
                        }
                        else
                        {
                            mLED_Ble(2);
                        }
                    }
                }
            }
            else if(data[0] == 'E')
            {
                //Config wifi
                if(memcmp("ESAP:", data, 5) == 0) //这个ESAP：命令只是用来做演示，后续需要修改带校验的命令
                {
                    p1 = (char*)&data[5];
                    for(p2=p1;(p2-p1)<rxBytes;p2++)
                    {
                        if(*p2==',')
                            break;
                    }

                    memset(u_config.SSID, 0, sizeof(u_config.SSID));
                    memset(u_config.Password, 0, sizeof(u_config.Password));

                    memcpy(u_config.SSID, p1, (p2-p1));
                    p2++;
                    memcpy(u_config.Password, p2, rxBytes-(p2-(char*)data));

                    if(Save_User_SSID_Password() == ESP_OK)
                    {
                       wifi_config_t wifi_config = {
                        .sta = {
                            .ssid = "123",
                            .password = "123",
                            },
                        };
                        if(b_wifi_is_connected)
                            ESP_ERROR_CHECK(esp_wifi_disconnect());

                        memcpy(wifi_config.sta.ssid, u_config.SSID, sizeof(wifi_config.sta.ssid));
                        memcpy(wifi_config.sta.password, u_config.Password, sizeof(wifi_config.sta.password));

                        ESP_LOGI(TAG, "Setting WiFi configuration SSID: %s...", wifi_config.sta.ssid);
                        ESP_LOGI(TAG, "Password: %s...", wifi_config.sta.password);
                        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
                        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
                        ESP_ERROR_CHECK( esp_wifi_start() );

                        /*if(b_wifi_is_connected == false)
                            esp_wifi_connect();*/

                        mLED_Wifi(2);
                    }
                }
            }
        }
    }
    free(data);
}


#define P_LED_RED       25
#define P_LED_BLUE      33
#define P_LED_GREEN     32
#define ALL_LED_PIN  ((1ULL<<P_LED_RED) | (1ULL<<P_LED_BLUE) | (1ULL<<P_LED_GREEN))

#define LED_RED(s)      gpio_set_level(P_LED_RED, !s)
#define LED_GREEN(s)    gpio_set_level(P_LED_GREEN, !s)
#define LED_BLUE(s)     gpio_set_level(P_LED_BLUE, !s)

typedef struct{
	uint8_t cnt;
	uint8_t on;
	uint8_t period;
	bool 			b_sw;
}my_led_single;

struct{
    my_led_single	r;
    my_led_single	g;
    my_led_single   b;
}my_led;

static void mled_task(void* arg)
{
    

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ALL_LED_PIN;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    mLED_Power(1);
    mLED_Ble(0);

    while(1)
    {
        vTaskDelay(250 / portTICK_RATE_MS);

        if(my_led.r.b_sw)
        {
            my_led.r.cnt++;
            if(my_led.r.cnt == my_led.r.on)
            {
                LED_RED(0);
            }
            else if(my_led.r.cnt >= my_led.r.period)
            {
                my_led.r.cnt = 0;
                LED_RED(1);
            }
        }
        
        if(my_led.g.b_sw)
        {
            my_led.g.cnt++;
            if(my_led.g.cnt == my_led.g.on)
            {
                LED_GREEN(0);
            }
            else if(my_led.g.cnt >= my_led.g.period)
            {
                my_led.g.cnt = 0;
                LED_GREEN(1);
            }
        }
        
        if(my_led.b.b_sw)
        {
            my_led.b.cnt++;
            if(my_led.b.cnt == my_led.b.on)
            {
                LED_BLUE(0);
            }
            else if(my_led.b.cnt >= my_led.b.period)
            {
                my_led.b.cnt = 0;
                LED_BLUE(1);
            }
        }
    }
}

void app_main()
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    initialise_wifi();
    uart_init();

    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, NULL, 5, NULL, 1);
    xTaskCreate(rx1_task, "uart_rx1_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(rx2_task, "uart_rx2_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //start gpio task
    xTaskCreate(mled_task, "mled_task", 2048, NULL, 10, NULL);
}




void mLED_RED(uint16_t on, uint16_t period)
{
	my_led.r.on = on / 25;
	my_led.r.period = period / 25;
	
	if(period)
	{
		if(my_led.r.b_sw == false)
		{
			my_led.r.cnt = my_led.r.period;
		}
		my_led.r.b_sw = true;
	}
	else
	{
		if(on)
			LED_RED(1);
		else
			LED_RED(0);
		
		my_led.r.b_sw = false;
	}
}

void mLED_GREEN(uint16_t on, uint16_t period)
{
	my_led.g.on = on / 25;
	my_led.g.period = period / 25;
	
	if(period)
	{
		if(my_led.g.b_sw == false)
		{
			my_led.g.cnt = my_led.g.period;
		}
		my_led.g.b_sw = true;
	}
	else
	{
		if(on)
			LED_GREEN(1);
		else
			LED_GREEN(0);
		
		my_led.g.b_sw = false;
	}
}

void mLED_BLUE(uint16_t on, uint16_t period)
{
	my_led.b.on = on / 25;
	my_led.b.period = period / 25;
	
	if(period)
	{
		if(my_led.b.b_sw == false)
		{
			my_led.b.cnt = my_led.b.period;
		}
		my_led.b.b_sw = true;
	}
	else
	{
		if(on)
			LED_BLUE(1);
		else
			LED_BLUE(0);
		
		my_led.b.b_sw = false;
	}
}

void mLED_Power(bool sta)
{
    mLED_RED(sta,0);
}

void mLED_Ble(uint8_t sta)
{
    Ble_Led_Status = sta;
    switch(sta)
    {
        case 0: mLED_BLUE(0, 0); break;
        case 1: mLED_BLUE(50, 100); break;
        case 2: mLED_BLUE(25, 50); break;
    }
}

void mLED_Wifi(uint8_t sta)
{
    Wifi_Led_Status = sta;
    switch(sta)
    {
        case 0: mLED_GREEN(0, 0); break;
        case 1: mLED_GREEN(50, 100); break;
        case 2: mLED_GREEN(25, 50); break;
    }
}


esp_err_t Read_User_Config(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    memset(&u_config, 0, sizeof(u_config));

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read User config blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, "user_config", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    if (required_size == 0) {
        printf("User config Not saved yet!\n");
    } else {
        err = nvs_get_blob(my_handle, "user_config", &u_config, &required_size);
        if (err != ESP_OK) {
            return err;
        }
    }

    // Close
    nvs_close(my_handle);

    return ESP_OK;
}

esp_err_t Save_User_SSID_Password(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "user_config", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    required_size = sizeof(u_config);
    err = nvs_set_blob(my_handle, "user_config", &u_config, required_size);
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}


