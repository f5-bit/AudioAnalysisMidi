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
	AudioFrequencyMeterMultiplexed(int s0, int s1, int s2);

	float getFrequencyMux(int muxPin);

	void begin(int pin, unsigned int sampleRate);
	void end(void);

	void setClippingPin(int pin);
	void checkClipping(void);

	void setAmplitudeThreshold(int threshold);
	void setTimerTolerance(int tolerance);
	void setSlopeTolerance(int tolerance);
	void setBandwidth(float minFrequency, float maxFrequency);

	float getFrequency(void);

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
	// For multiplexer
	int _s0;
	int _s1;
	int _s2;

private:
	int samplePin;           // Pin used to sample the signal
	unsigned int sampleRate; // ADC sample rate

	int amplitudeThreshold;

	float minFrequency;      // Variable to store the minimum frequency that can be applied in input
	float maxFrequency;      // Variable to store the maximum frequency that can be applied in input

};