#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MAX7219.h>
#include "compass.h"

extern "C" {
   void app_main();
}

/*
 * PINOUT
 * MOSI 13
 * MISO 12
 * CLK 14
 * CS 15
 * GPIO19 Button PullUp
 * GPIO23 Button PullUp
 */

#define S_DEVICE_INFO 0x180A
#define C_NAME_RX 0x2A29
#define C_SYSTEMID_RX 0x2A23

#define S_DISPLAY_INFO 0x1830
#define C_SPEED_Tx 0x2A59
#define C_DISTANCE_Tx 0x2A5A
#define C_HEADING_Tx 0x2A5B

const uint8_t NUMBERS[] = {
3,8, 0b00011111, 0b00010001, 0b00011111,
3,8, 0b00001000, 0b00011111, 0b00000000,
3,8, 0b00010111, 0b00010101, 0b00011101,
3,8, 0b00010101, 0b00010101, 0b00011111,
3,8, 0b00011100, 0b00000100, 0b00011111,
3,8, 0b00011101, 0b00010101, 0b00010111,
3,8, 0b00011111, 0b00010101, 0b00010111,
3,8, 0b00010000, 0b00010000, 0b00011111,
3,8, 0b00011111, 0b00010101, 0b00011111,
3,8, 0b00011100, 0b00010100, 0b00011111};

BLEServer *pServer;

SPI *spi_conn;
MAX7219 *matrix;

bool clientConnected = false;
const uint8_t ID[8]={0x12,0x31,0x23,0x01,0x02,0x03,0x04,0x05};

bool toShow = false;

bool toShowDistance = false;
uint8_t distance = 0;
bool toShowSpeed = false;
uint8_t speed = 0;
bool toShowHeading = false;
uint8_t heading = 0;

int state = 0;

static xQueueHandle gpio_evt_queue = NULL; // Queue to send button events

// Interruption handler for buttons
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
	  clientConnected = true;
	  printf("Connected\n");
	};

	void onDisconnect(BLEServer* pServer) {
	  clientConnected = false;
	  printf("Disonnected\n");
	}
};

class MyCallbackSPEED: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string aux_msj = pCharacteristic->getValue();
      if (aux_msj.length() > 0) {
      	speed = aux_msj.c_str()[0];
      	toShowSpeed = true;
      }
    }
};

class MyCallbackHEADING: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string aux_msj = pCharacteristic->getValue();
      if (aux_msj.length() > 0) {
      	heading = aux_msj.c_str()[0];
      	toShowHeading = true;
      }
    }
};

class MyCallbackDISTANCE: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string aux_msj = pCharacteristic->getValue();
      if (aux_msj.length() > 0) {
      	distance = aux_msj.c_str()[0];
      	toShowDistance = true;
      }
    }
};

void create_service_display()
{
	// create the BLE service 
	BLEService *deviceDisplay = pServer->createService(BLEUUID((uint16_t)S_DISPLAY_INFO));
	BLECharacteristic *DisplayChar;
	// Add characteristic
	DisplayChar = deviceDisplay->createCharacteristic(
	  BLEUUID((uint16_t)C_SPEED_Tx), BLECharacteristic::PROPERTY_WRITE);
	DisplayChar->setCallbacks(new MyCallbackSPEED());

	DisplayChar = deviceDisplay->createCharacteristic(
	  BLEUUID((uint16_t)C_DISTANCE_Tx), BLECharacteristic::PROPERTY_WRITE);
	DisplayChar->setCallbacks(new MyCallbackDISTANCE());

	DisplayChar = deviceDisplay->createCharacteristic(
	  BLEUUID((uint16_t)C_HEADING_Tx), BLECharacteristic::PROPERTY_WRITE);
	DisplayChar->setCallbacks(new MyCallbackHEADING());
	// Start the service
	deviceDisplay->start();
}

void create_service_device_info()
{
	// create the BLE service 
	BLEService *deviceInfo = pServer->createService(BLEUUID((uint16_t)S_DEVICE_INFO));
	// Add characteristic
	BLECharacteristic *ManufacName = deviceInfo->createCharacteristic(
	  BLEUUID((uint16_t)C_NAME_RX), BLECharacteristic::PROPERTY_READ );
	ManufacName->setValue("ESP32");
	// Add characteristic
	BLECharacteristic *systemID = deviceInfo->createCharacteristic(
	   BLEUUID((uint16_t)C_SYSTEMID_RX), BLECharacteristic::PROPERTY_READ);
	systemID->setValue((uint8_t *)ID,8);
	// Start the service
	deviceInfo->start();
}


void init()
{
	printf("Initializing BLE\n");
	BLEDevice::init("ESP32 DISPLAY");
	// Create the BLE Server
	printf("Creating BLE Server\n");
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());
	printf("Initializing characteristics\n");
	create_service_device_info();
	create_service_display();
	// Start advertising
	pServer->getAdvertising()->start();

	spi_conn = new SPI();
	spi_conn->init();
	matrix = new MAX7219(spi_conn, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	matrix->shutdown(true);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	matrix->shutdown(false);
	matrix->setIntensity(4);
	// Buttons
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interruption on falling edge
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_SEL_19 | GPIO_SEL_23;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Pull-up enabled
	gpio_config(&io_conf);
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));//Queue to handle events
	//install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_NUM_19, gpio_isr_handler, (void*) GPIO_NUM_19);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_NUM_23, gpio_isr_handler, (void*) GPIO_NUM_23);

	printf("Ready...\n");
}

void display_number_3(int number, int index)
{
	matrix->clearDisplay(0);
	int ind = 5*((number/100)%10);
	int i,d;
	if (ind == 5) {
		for (i = 0; i<2; i++) {
			matrix->setColumn(i, NUMBERS[ind+2+i]);
		}
	}
	ind = 5*((number/10)%10);
	if (ind >= 5) {
		for (i=2,d=0; i<5; i++,d++) {
			matrix->setColumn(i, NUMBERS[ind+2+d]);
		}
	}
	ind = 5*(number%10);
	for (i=5,d=0; i<8; i++,d++) {
		matrix->setColumn(i, NUMBERS[ind+2+d]);
	}
	matrix->setLed(0,index,true);
}

void display_number_2(int number, int index)
{
	matrix->clearDisplay(0);
	int i,d;
	int ind = 5*((number/10)%10);
	if (ind >= 5) {
		for (i=1,d=0; i<4; i++,d++) {
			matrix->setColumn(i, NUMBERS[ind+2+d]);
		}
	}
	ind = 5*(number%10);
	for (i=5,d=0; i<8; i++,d++) {
		matrix->setColumn(i, NUMBERS[ind+2+d]);
	}
	matrix->setLed(0,index,true);
}

void display_heading(float heading, int index)
{
	matrix->clearDisplay(0);
	matrix->setLed(0,index,true);

	matrix->setLed(4,3,true);

	if(heading < 22.5 || heading > 360-22.5) { // N
		matrix->setLed(3,3,true);
		matrix->setLed(2,3,true);
	} else if(heading > 22.5 && heading < 67.5) { // NW
		matrix->setLed(3,4,true);
		matrix->setLed(2,5,true);
	} else if(heading > 67.5 && heading < 112.5) { // W
		matrix->setLed(4,4,true);
		matrix->setLed(4,5,true);
	} else if(heading > 112.5 && heading < 157.5) { // SW
		matrix->setLed(5,4,true);
		matrix->setLed(6,5,true);
	} else if(heading > 157.5 && heading < 202.5) { // S
		matrix->setLed(5,3,true);
		matrix->setLed(6,3,true);
	} else if(heading > 202.5 && heading < 247.5) { // SE
		matrix->setLed(5,2,true);
		matrix->setLed(6,1,true);
	} else if(heading > 247.5 && heading < 292.5) { // E
		matrix->setLed(4,2,true);
		matrix->setLed(4,1,true);
	} else {
		matrix->setLed(3,2,true);
		matrix->setLed(2,1,true);
	}
}

void ble_task(void *pvParameter)
{
	init();
	init_compass();
	uint32_t io_num;
	int16_t x,y,z;
	float headinng;
	bool refresh = true;
	while(1) {
		if(xQueueReceive(gpio_evt_queue, &io_num, 0)) {
            if (io_num == GPIO_NUM_19) {
            	state = state - 1;
            	if (state < 0) state = 2;
            	printf("%d\n", state);
            	refresh = true;
            } else if (io_num == GPIO_NUM_23) {
            	printf("%d\n", state);
            	state = (state + 1) % 3;
            	refresh = true;
            }
        }
		switch (state) {
			case 0:
				read_compass_values(&y,&x,&z);
				headinng = atan2(x,-y);
				headinng += 6.161; // Declinación 7 grados este
				// Correct for headinng < 0deg and headinng > 360deg
				if (headinng < 0) {
					headinng += 2 * M_PI;
				}
				if (headinng > 2 * M_PI) {
					headinng -= 2 * M_PI;
				}
				headinng = headinng * 180/M_PI;
				headinng = headinng - (heading*360)/256;
				if (headinng < 0) {
					headinng += 360;
				}
				if (headinng > 360) {
					headinng -= 360;
				}
				printf("%f\n", headinng);
				display_heading(headinng,0);
				break;
			case 1:
				if (toShowDistance || refresh) {
					toShowDistance = false;
					refresh = false;
					display_number_3(distance,1);
				}
				break;
			case 2:
				if (toShowSpeed || refresh) {
					toShowSpeed = false;
					refresh = false;
					display_number_2(speed,2);
				}
				break;
		}
		xQueueReset(gpio_evt_queue);
  		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

void app_main()
{
	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	xTaskCreate(&ble_task, "ble_task", 2048, NULL, 5, NULL);
}