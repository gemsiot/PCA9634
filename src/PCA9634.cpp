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

#include "PCA9634.h"

PCA9634::PCA9634(int _ADR)
{
  ADR = _ADR; 
}

int PCA9634::begin(void) 
{
	//Only use isEnabled() if using particle
	#if defined(ARDUINO) && ARDUINO >= 100 
		Wire.begin();
	#elif defined(PARTICLE)
		if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	#endif

	return sleep(false); //Turn off sleep so device operated. Return I2C status, just to check if device is connected to bus 
}

//Turn the sleep state of the device on and off
int PCA9634::sleep(bool State)
{
	uint8_t Val = readByte(MODE1); //Read in current state of MODE1 reg
	if(State) Val = setBit(Val, 4); //Set the sleep bit
	if(!State) Val = clearBit(Val, 4); //Clear the sleep bit
	return writeByte(MODE1, Val); //Write adjusted value back to register   
}

int PCA9634::setOutputMode(OutputMode State)
{
	uint8_t Val = readByte(MODE2); //Read in current value of MODE2 register
	if(State == OpenDrain) Val = clearBit(Val, 2); //Clear OUTDRV bit for open drain output
	if(State == TotemPole) Val = setBit(Val, 2); //Set OUTDRV bit for totem pole output
	return writeByte(MODE2, Val); //Write adjusted value back
}

int PCA9634::setGroupMode(GroupMode State)
{
	uint8_t Val = readByte(MODE2); //Read in current state of MODE2 reg
	if(State == Dim) Val = clearBit(Val, 5); //Clear bit if wanting to set bank for dimming
	if(State == Blink) Val = setBit(Val, 5); //Set bit if want to set back for blinking
	return writeByte(MODE2, Val); //Write adjusted register back
}

int PCA9634::setGroupBlinkFreq(float Freq) //Set the frequency [Hz] of the group blinking on and off. Must be set to Blink mode to be effective (same control as setGroupBlinkPeriod())
{

	if(Freq > MAX_FREQ) {
		writeByte(GRPFREQ, 0x00); //Set to max frequency
		return -1; //Indicate error in input condition
	}
	if(Freq < MIN_FREQ) {
		writeByte(GRPFREQ, 0xFF); //Set to min frequency
		return -1; //Indicate error in input condition
	}

	uint8_t RegVal = int((MAX_FREQ/Freq) - 1.0); //Round to nearest frequency 
	BlinkPeriod = ((RegVal + 1)*1000)/24; //Back caulculate the commanded period in ms, store
	return writeByte(GRPFREQ, RegVal); //Write value back to register
}

int PCA9634::setGroupBlinkPeriod(uint16_t Period) //Set the period [ms] of the group blinking on and off. Must be set to Blink mode to be effective (same control as setGroupBlinkFreq())
{
	if(Period > MAX_PERIOD) {
		writeByte(GRPFREQ, 0xFF); //Set to max period
		BlinkPeriod = MAX_PERIOD; //Set to max value
		return -1; //Indicate error in input condition
	}
	if(Period < MIN_PERIOD) {
		writeByte(GRPFREQ, 0x00); //Set to min period
		BlinkPeriod = MIN_PERIOD; //Set to min value
		return -1; //Indicate error in input condition
	}

	uint8_t RegVal = int(((float(Period)/1000.0)*MAX_FREQ) - 1.0); //Round to nearest period 
	BlinkPeriod = ((RegVal + 1)*1000)/24; //Back caulculate the commanded period in ms, store
	return writeByte(GRPFREQ, RegVal); //Write value back to register
}

int PCA9634::setGroupDutyCycle(float Duty) //Set the duty cycle [%] of the time the group is on. Must be set to Blink mode to be effective 
{
	Duty = Duty/100.0; //Convert to 0 ~ 1 range
	if(Duty > MAX_DUTY) {
		writeByte(GRPPWM, 0xFF); //Set to max on time
		return -1; //Indicate error in input condition
	}
	if(Duty < MIN_DUTY) {
		writeByte(GRPPWM, 0x00); //Set to min on time
		return -1; //Indicate error in input condition
	}
	uint8_t RegVal = int(Duty*256.0); //Calculate duty cycle to nearest int
	return writeByte(GRPPWM, RegVal); //Write value to PWM register
}

int PCA9634::setGroupOnTime(uint16_t Period) //Set the period [ms] of the time the group is on. Must be set to Blink mode to be effective 
{
	if(Period > BlinkPeriod*MAX_DUTY) { //If greater than possible on period in ms
		writeByte(GRPPWM, 0xFF); //Set to max on time
		return -1; //Indicate error in input condition
	}
	if(Period < 0) { //If less than 0ms
		writeByte(GRPPWM, 0x00); //Set to min on time
		return -1; //Indicate error in input condition
	}
	uint8_t RegVal = int((float(Period)/BlinkPeriod)*256.0); //Calculate duty cycle to nearest int by convering it to duty cycle relative to current commanded BlinkPeriod
	return writeByte(GRPPWM, RegVal); //Write value to PWM register
}

int PCA9634::setGroupBrightness(float Brightness) //Set the brightness output of the group [%]. Must be set to Dim mode to be effective
{
	//FIX! Test if group mode is set correctly?? Should we do things this way or just let user handle things?	
	Brightness = Brightness/100.0; //Convert to 0 ~ 1 range
	if(Brightness > MAX_DUTY) {
		writeByte(GRPPWM, 0xFF); //Set to max on time
		return -1; //Indicate error in input condition
	}
	if(Brightness < MIN_DUTY) {
		writeByte(GRPPWM, 0x00); //Set to min on time
		return -1; //Indicate error in input condition
	}
	uint8_t RegVal = int(Brightness*256.0); //Calculate duty cycle to nearest int
	return writeByte(GRPPWM, RegVal); //Write value to PWM register
}

int PCA9634::setBrightness(uint8_t Pos, float Brightness) //Set brightness [%] for a specific port
{
	Brightness = Brightness/100.0; //Convert to 0 ~ 1 range
	if(Brightness > MAX_DUTY) {
		writeByte(PWM0 + Pos, 0xFF); //Set specified port to max on time
		return -1; //Indicate error in input condition
	}
	if(Brightness < MIN_DUTY) {
		writeByte(PWM0 + Pos, 0x00); //Set specified port to min on time
		return -1; //Indicate error in input condition
	}
	uint8_t RegVal = int(Brightness*256.0); //Calculate duty cycle to nearest int
	return writeByte(PWM0 + Pos, RegVal); //Write value to specified port PWM register
}

int PCA9634::setBrightnessArray(float Brightness) //Set all ports to same brightness value
{
	//FIX! Find more elegant solution?
	int Error = 0;
	for(int i = 0; i < 8; i++) { //Interate over all ports and set value
		if(setBrightness(i, Brightness) != 0) Error = -1; //FIX! Use different error value? 
	}
	return Error; //Return cumulative error state
}

int PCA9634::setOutput(uint8_t Pos, PortState State) //Set state (Off, On, PWM, Group) of a given port
{
	//Off - Fully off 
	//On - Fully on
	//PWM - On, but brightness controlled by PWMx register
	//Group - On, but brightness and group dimming/blinking controlled by PWMx and GRPPWM registers
	if(Pos > 7) return -1; //Indicate input error if out of range

	uint16_t ControlReg = (readByte(LEDOUT0 + 1) << 8) | readByte(LEDOUT0); //Concatonate control registers into one array for ease of access
	ControlReg = ControlReg & ~(0x0003 << (Pos*2)); //Clear bits in question
	ControlReg = ControlReg | (State << Pos*2); //Set bits for specified port

	writeByte(LEDOUT0, ControlReg & 0xFF); //Write low byte back
	return writeByte(LEDOUT0 + 1, ControlReg >> 8); //Write high byte back
}

int PCA9634::setOutputArray(PortState State) //Set all ports to same value 
{
	//FIX! Find more elegant solution?
	int Error = 0;
	for(int i = 0; i < 8; i++) { //Interate over all ports and set value
		if(setOutput(i, State) != 0) Error = -1; //FIX! Use different error value? 
	}
	return Error; //Return cumulative error state
}

int PCA9634::digitalWrite(uint8_t Pin, bool State) //Arduino compatable digital write function
{
	//Fix! Make more elegant 
	PortState Val = Off;
	if(State == 0) Val = Off;
	if(State == 1) Val = On;
	return setOutput(Pin, Val); //Pass value along to setOutput function, either turn off or on
}

int PCA9634::analogWrite(uint8_t Pin, uint8_t Value) //Arduin compatable analog write function (0~255 PWM)
{
	setBrightness(Pin, (Value*100.0)/255.0); //Convert PWM value and set brightness control
	return setOutput(Pin, PWM); //Set output to be PWM controlled 
}

/////////////// HELPER FUNCTIONS ////////////////////////

int PCA9634::writeByte(uint8_t Reg, uint8_t Data)
{
	Wire.beginTransmission(ADR);
	Wire.write(Reg);
	Wire.write(Data);
	return Wire.endTransmission();
}

uint8_t PCA9634::readByte(uint8_t Reg)
{
	Wire.beginTransmission(ADR);
	Wire.write(Reg);
	int Error = Wire.endTransmission();

	Wire.requestFrom(ADR, 1);
	uint8_t Data = Wire.read(); //FIX! Add wait for result??

	if(Error == 0) return Data;
	else return 0; //FIX! Find better return method??
}

uint8_t PCA9634::clearBit(uint8_t Val, uint8_t Pos)
{
  return Val & ~(0x01 << Pos); //Return adjusted byte
}

uint8_t PCA9634::setBit(uint8_t Val, uint8_t Pos)
{
	return Val | (0b01 << Pos); //Return adjusted byte
}
