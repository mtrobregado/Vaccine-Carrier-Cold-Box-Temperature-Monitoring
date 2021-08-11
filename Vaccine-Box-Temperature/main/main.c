/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Cloud Connected Blinky v1.3.1
 * main.c
 * 
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
 * @file main.c
 * @brief simple MQTT publish and subscribe for use with AWS IoT EduKit reference hardware.
 *
 * This example takes the parameters from the build configuration and establishes a connection to AWS IoT Core over MQTT.
 *
 * Some configuration is required. Visit https://edukit.workshop.aws
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "cJSON.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "core2forAWS.h"

#include "wifi.h"
#include "blink.h"
#include "ui.h"

/* The time between each MQTT message publish in milliseconds */
#define PUBLISH_INTERVAL_MS 3000

#define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))
#define CELSIUS_PER_LSB 0.0025177F

/* The time prefix used by the logger. */
static const char *TAG = "MAIN";

/* The FreeRTOS task handler for the blink task that can be used to control the task later */
TaskHandle_t xBlink;

/* CA Root certificate */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");


/*! UART Data message: sent from the collector to AWS IOT EduKit */
 
typedef struct _uart_sensormsg_t
{
  uint16_t nodecnt;
  uint16_t containerid;
  uint8_t vaccinename[20];
  uint16_t temp;
  uint16_t battval;

}uart_sensorMsg_t;

uart_sensorMsg_t uartmsg;
static int mintemp = 2;
static int maxtemp = 25;
static double dtemp = 0;

/* Default MQTT HOST URL is pulled from the aws_iot_config.h */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/* Default MQTT port is pulled from the aws_iot_config.h */
uint32_t port = AWS_IOT_MQTT_PORT;

double f_round(double dval, int n)
{
    char l_fmtp[32], l_buf[64];
    char *p_str;
    sprintf (l_fmtp, "%%.%df", n);
    if (dval>=0)
            sprintf (l_buf, l_fmtp, dval);
    else
            sprintf (l_buf, l_fmtp, dval);
    return ((double)strtod(l_buf, &p_str));

}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
    if (strstr(topicName, "/temprange") != NULL) {
     
        cJSON *item;       
        cJSON *root = cJSON_Parse((char *)params->payload);
             
        item = cJSON_GetObjectItem(root, "min");
        mintemp = *(&(item->valueint));             
       
        item = cJSON_GetObjectItem(root, "max");
        maxtemp = *(&(item->valueint));        

        cJSON_Delete(root);

        ui_textarea_add_number("\nnew min temp: %d\n", mintemp, sizeof(int));    
        ui_textarea_add_number("new max temp: %d\n", maxtemp, sizeof(int));      
    }
}

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    ui_textarea_add("Disconnected from AWS IoT Core...", NULL, 0);
    IoT_Error_t rc = FAILURE;

    if(pClient == NULL) {
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

static void publisher(AWS_IoT_Client *client, char *base_topic, uint16_t base_topic_len){
  
    IoT_Publish_Message_Params paramsQOS0; 
    paramsQOS0.qos = QOS0;    
    paramsQOS0.isRetained = 0;    
    
    dtemp = (double)((uartmsg.temp * CELSIUS_PER_LSB) - 40);   
    dtemp = f_round(dtemp, 2);  

    // Get state of the FreeRTOS task, "blinkTask", using it's task handle.
    eTaskState blinkState = eTaskGetState(xBlink);

    if((dtemp < mintemp) || (dtemp > maxtemp)){        
        if (blinkState == eSuspended){
            vTaskResume(xBlink);
        } 
    }
    else if((dtemp >= mintemp) && (dtemp <= maxtemp)){        
        vTaskSuspend(xBlink);  
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, 0x000000);
        Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_RIGHT, 0x000000);
        Core2ForAWS_Sk6812_Show();             
    }

    cJSON *payload = cJSON_CreateObject();
    cJSON *container_id = cJSON_CreateNumber(uartmsg.containerid);
    cJSON *vaccine_name = cJSON_CreateString((const char *)uartmsg.vaccinename);    
    cJSON *temperature = cJSON_CreateNumber(dtemp);
    cJSON *batt_value = cJSON_CreateNumber(uartmsg.battval);

    cJSON_AddItemToObject(payload, "container_id", container_id);
    cJSON_AddItemToObject(payload, "vaccine_name", vaccine_name);
    cJSON_AddItemToObject(payload, "temperature", temperature);
    cJSON_AddItemToObject(payload, "battery_value", batt_value);

    const char *JSONPayload = cJSON_Print(payload);
    paramsQOS0.payload = (void *) JSONPayload;
    paramsQOS0.payloadLen = strlen(JSONPayload);

    // Publish and ignore if "ack" was received or  from AWS IoT Core  
    IoT_Error_t rc = aws_iot_mqtt_publish(client, base_topic, base_topic_len, &paramsQOS0);
    if (rc != SUCCESS){
        ESP_LOGE(TAG, "Publish QOS0 error %i", rc);
        rc = SUCCESS;
    }

    ui_textarea_add("%s", (char *)JSONPayload, paramsQOS0.payloadLen);
    cJSON_Delete(payload);
}

void aws_iot_task(void *param) {
    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;    
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = "#";
    mqttInitParams.pDevicePrivateKeyLocation = "#0";
    
#define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
#define SUBSCRIBE_TOPIC_LEN (CLIENT_ID_LEN + 3)
#define BASE_PUBLISH_TOPIC_LEN (CLIENT_ID_LEN + 2)

    char *client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS)
    {
        printf("Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    char subscribe_topic[SUBSCRIBE_TOPIC_LEN];
    char base_publish_topic[BASE_PUBLISH_TOPIC_LEN];
    snprintf(subscribe_topic, SUBSCRIBE_TOPIC_LEN, "%s/#", client_id);
    snprintf(base_publish_topic, BASE_PUBLISH_TOPIC_LEN, "%s/", client_id);

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnect_callback_handler;
    mqttInitParams.disconnectHandlerData = NULL;

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

    connectParams.pClientID = client_id;
    connectParams.clientIDLen = CLIENT_ID_LEN;
    connectParams.isWillMsgPresent = false;
    ui_textarea_add("Connecting to AWS IoT Core...\n", NULL, 0);
    ESP_LOGI(TAG, "Connecting to AWS IoT Core at %s:%d", mqttInitParams.pHostURL, mqttInitParams.port);
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while(SUCCESS != rc);
    ui_textarea_add("Successfully connected!\n", NULL, 0);
    ESP_LOGI(TAG, "Successfully connected to AWS IoT Core!");

    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time for exponential backoff for retries.
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        ui_textarea_add("Unable to set Auto Reconnect to true\n", NULL, 0);
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "Subscribing to '%s'", subscribe_topic);
    rc = aws_iot_mqtt_subscribe(&client, subscribe_topic, strlen(subscribe_topic), QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        ui_textarea_add("Error subscribing\n", NULL, 0);
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    } else{
        ui_textarea_add("Subscribed to topic: %s\n\n", subscribe_topic, SUBSCRIBE_TOPIC_LEN) ;
        ESP_LOGI(TAG, "Subscribed to topic '%s'", subscribe_topic);
    }
    
    ESP_LOGI(TAG, "\n****************************************\n*  AWS client Id - %s  *\n****************************************\n\n",
             client_id);
    
    ui_textarea_add("Attempting publish to: %s\n", base_publish_topic, BASE_PUBLISH_TOPIC_LEN) ;
    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGD(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS));
        
        publisher(&client, base_publish_topic, BASE_PUBLISH_TOPIC_LEN);
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}
 
static void uart_rx_task(void *arg){
    int rxBytes;
    uint8_t *data = heap_caps_malloc(UART_RX_BUF_SIZE, MALLOC_CAP_SPIRAM); // Allocate space for message in external RAM
    
    
    while (1) {
        rxBytes = Core2ForAWS_Port_C_UART_Receive(data);
        if (rxBytes > 0) {

            memcpy(&uartmsg, data, sizeof(uartmsg));
            ESP_LOGI(TAG, "node cnt: %d, container id: %d, vaccine name: %s, temp: %.2d, batt: %d", uartmsg.nodecnt, uartmsg.containerid, 
                                                                                                    uartmsg.vaccinename, uartmsg.temp, 
                                                                                                    uartmsg.battval);
                                                                                        
            
        }
        vTaskDelay(pdMS_TO_TICKS(75)); 
    }
    free(data); // Free memory from external RAM
}

void app_main()
{
    Core2ForAWS_Init(); 

    esp_err_t err = Core2ForAWS_Port_PinMode(PORT_C_UART_TX_PIN, UART);
    if (err == ESP_OK){
        Core2ForAWS_Port_C_UART_Begin(115200);
        xTaskCreate(uart_rx_task, "uart_rx", 1024*2, NULL, configMAX_PRIORITIES, NULL);               
    }

    Core2ForAWS_Display_SetBrightness(80);
    
    ui_init();
    initialise_wifi();
   
    ui_textarea_add_number("\ndefault min temp: %d\n", mintemp, sizeof(int));    
    ui_textarea_add_number("default max temp: %d\n", maxtemp, sizeof(int));

    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 4096 * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&blink_task, "blink_task", 4096 * 1, NULL, 2, &xBlink, 1);
}