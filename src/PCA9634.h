/******************************************************************************
PCA9634
Interface for PCA9634 8 bit LED driver 
Bobby Schulz @ GEMS Sensing
10/29/2021
https://github.com/gemsiot/PCA9634

Allows control of individual LEDs on the driver as well as the dimming and blink rates of banks

0.0.0

"I predict that very shortly the old-fashioned incandescent lamp, having a filament heated to brightness by the passage of electric current through it, will entirely disappear."
-Nikola Tesla

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef PCA9634_h
#define PCA9634_h

#include "Arduino.h"
#include <Wire.h>

#define MODE1 0x00 //Mode1 register address
#define MODE2 0x01 //Mode2 register address
#define PWM0 0x02 //PWM0 control register address, increment from this for 0~7 registers
#define GRPPWM 0x0A //Group PWM control register address
#define GRPFREQ 0x0B //Group frequency control register address
#define LEDOUT0 0x0C //LED0 output control register, increment 1 to get to LEDOUT1 register 

#define MAX_FREQ 24.0 //Max frequency is 24Hz
#define MIN_FREQ 0.09375 //Min frequency is 0.09375 Hz 

#define MAX_PERIOD 10666 //Max period is 10.6667 seconds
#define MIN_PERIOD 41 //Min period if 41 ms

#define MAX_DUTY 0.996 //Maximum allowable duty cycle 
#define MIN_DUTY 0 //Minimum allowable duty cycle

enum GroupMode{
	Dim = 0,
	Blink = 1,
};

enum OutputMode{
	OpenDrain = 0,
	TotemPole = 1,
};

enum PortState{
	Off = 0,
	On = 1,
	PWM = 2,
	Group = 3,
};

class PCA9634
{
  public:

    PCA9634(int _ADR); 
    int begin(void);
    int sleep(bool State);
    int setOutputMode(OutputMode State);
    int setGroupMode(GroupMode State);
    int setGroupBlinkFreq(float Freq);
    int setGroupBlinkPeriod(uint16_t Period);
    int setGroupDutyCycle(float Duty);
    int setGroupOnTime(uint16_t Period);

    int setGroupBrightness(float Brightness);
    int setBrightness(uint8_t Pos, float Brightness);

    int setOutput(uint8_t Pos, PortState State);
    int setOutputArray(PortState Val);

    // Arduino compatable functions
    int digitalWrite(uint8_t Pin, bool State);
    int analogWrite(uint8_t Pin, uint8_t Value);


  private:
    int ADR;

    int writeByte(uint8_t Reg, uint8_t Data);
    uint8_t readByte(uint8_t Reg);
    uint8_t clearBit(uint8_t Val, uint8_t Pos);
    uint8_t setBit(uint8_t Val, uint8_t Pos);
    uint16_t BlinkPeriod = 0; //Used to keep track of the commanded period for the blink [ms]

};

#endif