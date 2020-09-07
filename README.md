# Audio Analysis Midi for Arduino Zero

Allows the Arduino Zero and MKR1000 to sample several generic input audio signals with an analogue multiplexer, get the fundamental pitch for each multiplexed signal and output the corresponding MIDI signal.

Based on
* [Audio Frequency Meter Library](https://github.com/arduino-libraries/AudioFrequencyMeter)
* Arduino sketch [AnalogToMidi](https://github.com/arduino-libraries/AudioFrequencyMeter) by Arturo Guadalupi
* [Analog Multiplexer/Demultiplexer - 4051](https://playground.arduino.cc/Learning/4051/) by David C., Tomek N., Ross R. and Igor de Oliveira Sá.


## Methods
* begin(uint32_t ulPin, uint32_t sampleRate) : initialize the ADC so sample ulPin at the chosen sample rate. This process works in interrupt using TC5 to start the sampling process. ADC resolution is set to 8 bit

* end() : stops the sampling process disabling both the ADC and TC5 and resetting TC5

* setClippingPin(int pin) : put pin in output to be used as a clipping indicator

* checkClipping : checks if there is a clipping event (converted value equals to the top or the bottom of the ADC dynamic) and drives HIGH the clippingPin

* setAmplitudeThreshold(uint8_t threshold) sets the threshold for which a detected frequency is considered right or wrong. Default is 30

* setTimerTolerance(int tolerance)  : sets the tolerance for which a sampled signal is considered valid. Default is 10

* setSlopeTolerance(int tolerance) : sets the tolerance for which the slope is valid for the trigger process. Default is 3

* setBandwidth(float minFrequency, float maxFrequency) : set the range of frequencies for which the detected frequency is valid. Default values for now are 60Hz - 1500Hz. This must be improved 

* getFrequencyMux(int muxPin) : return the value of the detected frequency for the input signal at the selected multiplexer pin if it is above the threshold defined by setAmplitudeThreshold else -1

* getAnalogueData(int muxPin) : return the value of the ADC for the input signal at the selected multiplexer pin

* void setMultiplexer(int s0, int s1, int s2) : set the select pins for controlling the multiplexer (i.e. 74HCT 4051), see [Analog Multiplexer/Demultiplexer - 4051](https://playground.arduino.cc/Learning/4051/)
