
#pragma once
#include "AudioFrequencyMeter.h"

class AudioFrequencyMeterMultiplexed : public AudioFrequencyMeter {
public:
	AudioFrequencyMeterMultiplexed(int s0, int s1, int s2);

	float getFrequencyMux(int muxPin);

private:
	//void initializeVariables(void);
	void initializeVariables();
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