/***************************************************************************************
*  Name:	AudioFrequencyMeterMultiplexed.h
*  Date:	01.09.2020
*  Author:	Franca Bittner
***************************************************************************************/

/* Based on*/
/***************************************************************************************
*  Title:	Audio Frequency Meter Library for Arduino Zero
*  Authors:	Sandeep Mistry and Arturo Guadalupi
*  Date:	17.01.2017
*  Version:	1.0.3
*  Availability: https://github.com/arduino-libraries/AudioFrequencyMeter
***************************************************************************************/

#include "Arduino.h"
#include "wiring_private.h"

#pragma once

#define DEFAULT_AMPLITUDE_THRESHOLD     30
#define DEFAULT_TIMER_TOLERANCE         10
#define DEFAULT_SLOPE_TOLERANCE         3
#define DEFAULT_MIN_FREQUENCY           60.00
#define DEFAULT_MAX_FREQUENCY           1500.00

bool ADCisSyncing(void);
uint8_t ADCread();

class AudioFrequencyMeterMultiplexed {
public:
	AudioFrequencyMeterMultiplexed(); //(int s0, int s1, int s2);

	float getFrequencyMux(int muxPin);
	float getAnalogueData(int muxPin);
	void setMultiplexer(int s0, int s1, int s2);

	void begin(int pin, unsigned int sampleRate);
	void end(void);

	void setClippingPin(int pin);
	void checkClipping(void);

	void setAmplitudeThreshold(int threshold);
	void setTimerTolerance(int tolerance);
	void setSlopeTolerance(int tolerance);
	void setBandwidth(float minFrequency, float maxFrequency);

private:
	void initializeVariables(void);
	void ADCconfigure();
	void ADCenable(void);
	void ADCdisable(void);
	void ADCsetMux(void);

	void tcConfigure(void);
	bool tcIsSyncing(void);
	void tcEnable(void);
	void tcDisable(void);
	void tcReset(void);

public:

private:
	int samplePin;           // Pin used to sample the signal
	unsigned int sampleRate; // ADC sample rate

	int amplitudeThreshold;

	float minFrequency;      // Variable to store the minimum frequency that can be applied in input
	float maxFrequency;      // Variable to store the maximum frequency that can be applied in input
};