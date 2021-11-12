#include <PCA9634.h>

uint8_t Pin = 0; //Select which pin on the device to toggle
unsigned long BlinkPeriod = 1000; //Delay 1000ms between blinks

PCA9634 led(0x50); //Instantiate driver with address = 0x50

void setup() {
	led.begin();
}

void loop() {
	// static uint8_t State = 1; //Start as having LED on
	led.setOutput(Pin, On);
	delay(BlinkPeriod);
	led.setOutput(Pin, Off);
	delay(BlinkPeriod);
}