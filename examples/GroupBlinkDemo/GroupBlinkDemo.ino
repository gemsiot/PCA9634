#include <PCA9634.h>

PCA9634 led(0x29); //Instatiate LED driver with 0x29 address

const uint16_t Period = 1500; //1.5 second blink period
const uint16_t OnTime = 400; //400ms on time
const unsigned long SwitchPeriod = 10000; //Change blink types every 10 seconds

void setup()
{
	led.begin(); //Initalize LED controller 
	led.setOutputArray(Off); //Turn all off by default
	led.setGroupBrightness(50); //Set to 50% brightness
}

void loop()
{
	static bool State = 0;
	for(int i = 0; i < 4; i++) { //Set LEDs to alternate on and off
		led.setOutput(i*2 + State, Off);
		led.setOutput(i*2 + !State, Group);
	}
	State = !State; //Toggle state
	delay(SwitchPeriod); //Wait for the global switching period while the LEDs freely blink
}