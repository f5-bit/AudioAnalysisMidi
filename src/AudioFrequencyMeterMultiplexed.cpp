#include "AudioFrequencyMeterMultiplexed.h"

#define ARRAY_DEPTH             20
#define NOT_INITIALIZED         -1
#define TIMER_TIMEOUT           1000
#define BOTTOMPOINT             0
#define MIDPOINT                127
#define TOPPOINT                255

#define MUXSIZE					8

static int slopeTolerance;
static int timerTolerance;

static bool clipping;
static int clippingPin;

static int  newData[MUXSIZE], prevData;                           // Variables to store ADC result

static unsigned int time[MUXSIZE], totalTimer;                    // Variables used to compute period
static volatile unsigned int period[MUXSIZE];

static int arrayIndex[MUXSIZE];                                   // Index to save data in the correct position of the arrays
static int timer[ARRAY_DEPTH][MUXSIZE];                           // Array to store trigger events
static int slope[ARRAY_DEPTH][MUXSIZE];                           // Array to store changing in slope events

static int maxSlope[MUXSIZE];                                     // Variable to store max detected amplitude
static int newSlope;                                     // Variable to store a new slope

static int noMatch[MUXSIZE];                                      // Variable to store non-matching trigger events

static unsigned int amplitudeTimer[MUXSIZE];                      // Variable to reset trigger
static int maxAmplitude[MUXSIZE];                                 // Variable to store the max detected amplitude
static int newMaxAmplitude;                              // Variable used to check if maxAmplitude must be updated

static volatile int checkMaxAmp[MUXSIZE];                         // Used to update the new frequency in base of the AMplitude threshold

// For multiplexer

int r0 = 0;      //value of select pin at the 4051 (s0)
int r1 = 0;      //value of select pin at the 4051 (s1)
int r2 = 0;      //value of select pin at the 4051 (s2)

int currentMuxPin = 0;


AudioFrequencyMeterMultiplexed::AudioFrequencyMeterMultiplexed() {
	initializeVariables();
}

float AudioFrequencyMeterMultiplexed::getFrequencyMux(int muxPin)
{
	float frequency = -1;

	if (checkMaxAmp[muxPin] > amplitudeThreshold) {
		frequency = (float)(sampleRate / period[muxPin]);

		if ((frequency < minFrequency) || (frequency > maxFrequency)) {
			frequency = -1;
		}
	}

	return frequency;
}

void setMuxPinToRead(int muxPin)
{
	r0 = bitRead(muxPin, 0);   // use this with arduino 0013 (and newer versions)
	r1 = bitRead(muxPin, 1);   // use this with arduino 0013 (and newer versions)
	r2 = bitRead(muxPin, 2);   // use this with arduino 0013 (and newer versions)

	// TODO: replace hard-coded control pins by _s0, _s1 and _s2
	digitalWrite(2, r0);
	digitalWrite(3, r1);
	digitalWrite(4, r2);
}

void AudioFrequencyMeterMultiplexed::begin(int pin, unsigned int rate)
{
	samplePin = pin;                              // Store ADC channel to sample
	sampleRate = rate;                            // Store sample rate value
	setMuxPinToRead(0);
	analogRead(pin);                              // To start setting-up the ADC

	ADCdisable();
	ADCconfigure();
	ADCenable();
	tcConfigure();
	tcEnable();
}

void AudioFrequencyMeterMultiplexed::end()
{
	ADCdisable();
	tcDisable();
	tcReset();
}

void AudioFrequencyMeterMultiplexed::setClippingPin(int pin)
{
	clippingPin = pin;                              // Store the clipping pin value
	pinMode(clippingPin, OUTPUT);
}

void AudioFrequencyMeterMultiplexed::checkClipping()
{
	if (clipping) {
		digitalWrite(clippingPin, LOW);
		clipping = false;
	}
}

void AudioFrequencyMeterMultiplexed::setAmplitudeThreshold(int threshold)
{
	amplitudeThreshold = abs(MIDPOINT - threshold);
}

void AudioFrequencyMeterMultiplexed::setTimerTolerance(int tolerance)
{
	timerTolerance = tolerance;
}

void AudioFrequencyMeterMultiplexed::setSlopeTolerance(int tolerance)
{
	slopeTolerance = tolerance;
}

void AudioFrequencyMeterMultiplexed::setBandwidth(float min, float max)
{
	minFrequency = min;
	maxFrequency = max;
}
//
//float AudioFrequencyMeterMultiplexed::getFrequency()
//{
//	float frequency = -1;
//
//	if (checkMaxAmp > amplitudeThreshold) {
//		frequency = (float)(sampleRate / period);
//
//		if ((frequency < minFrequency) || (frequency > maxFrequency)) {
//			frequency = -1;
//		}
//	}
//
//	return frequency;
//}

/*
   Private Utility Functions
*/

void AudioFrequencyMeterMultiplexed::initializeVariables()
{
	slopeTolerance = DEFAULT_SLOPE_TOLERANCE;
	timerTolerance = DEFAULT_TIMER_TOLERANCE;
	amplitudeThreshold = DEFAULT_AMPLITUDE_THRESHOLD;

	clipping = false;
	clippingPin = NOT_INITIALIZED;

	for (int i = 0; i < MUXSIZE; i++)
	{
		newData[i] = 0;
		maxAmplitude[i] = 0;
		time[i] = 0;
		amplitudeTimer[i] = 0;
		maxSlope[i] = 0;
		noMatch[i] = 0;
		arrayIndex[i] = 0;
		checkMaxAmp[i] = 0;
	}

	prevData = MIDPOINT;
	minFrequency = DEFAULT_MIN_FREQUENCY;
	maxFrequency = DEFAULT_MAX_FREQUENCY;
}

void AudioFrequencyMeterMultiplexed::ADCconfigure()
{
	ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
	while (ADCisSyncing())
		;

	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val;     // Divide Clock by 8 -> ~100kHz
	while (ADCisSyncing())
		;


	while (ADCisSyncing())
		;

	ADC->SAMPCTRL.reg = 0x1F;                                    // Set max Sampling Time Length
	while (ADCisSyncing())
		;

	ADCsetMux();
}

bool ADCisSyncing()
{
	return (ADC->STATUS.bit.SYNCBUSY);
}

void AudioFrequencyMeterMultiplexed::ADCdisable()
{
	ADC->CTRLA.bit.ENABLE = 0x00;                               // Disable ADC
	while (ADCisSyncing())
		;
}

void AudioFrequencyMeterMultiplexed::ADCenable()
{
	ADC->CTRLA.bit.ENABLE = 0x01;                               // Enable ADC
	while (ADCisSyncing())
		;
}

void AudioFrequencyMeterMultiplexed::ADCsetMux()
{
	if (samplePin < A0)
	{
		samplePin += A0;
	}

	pinPeripheral(samplePin, g_APinDescription[samplePin].ulPinType);

	while (ADCisSyncing())
		;
	ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[samplePin].ulADCChannelNumber; // Selection for the positive ADC input
}

void AudioFrequencyMeterMultiplexed::tcConfigure()
{
	// Enable GCLK for TCC2 and TC5 (timer counter input clock)
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
	while (GCLK->STATUS.bit.SYNCBUSY);

	tcReset();

	// Set Timer counter Mode to 16 bits
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

	// Set TC5 mode as match frequency
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

	TC5->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / sampleRate - 1);
	while (tcIsSyncing())
		;

	sampleRate = SystemCoreClock / (TC5->COUNT16.CC[0].reg + 1);

	// Configure interrupt request
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0x00);
	NVIC_EnableIRQ(TC5_IRQn);

	// Enable the TC5 interrupt request
	TC5->COUNT16.INTENSET.bit.MC0 = 1;
	while (tcIsSyncing())
		;
}

bool AudioFrequencyMeterMultiplexed::tcIsSyncing()
{
	return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void AudioFrequencyMeterMultiplexed::tcEnable()
{
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	while (tcIsSyncing())
		;
}

void AudioFrequencyMeterMultiplexed::tcReset()
{
	TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (tcIsSyncing())
		;
	while (TC5->COUNT16.CTRLA.bit.SWRST)
		;
}

void AudioFrequencyMeterMultiplexed::tcDisable()
{
	// Disable TC5
	TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (tcIsSyncing())
		;
}

uint8_t ADCread()
{
	uint8_t returnValue;

	while (ADCisSyncing())
		;

	ADC->SWTRIG.bit.START = 1;

	while (ADC->INTFLAG.bit.RESRDY == 0);   					// Waiting for conversion to complete
	returnValue = ADC->RESULT.reg;            					// Store the value

	while (ADCisSyncing())
		;

	ADC->SWTRIG.bit.START = 0;

	return returnValue;
}

void analyzeIncomingData(int index)
{
	prevData = newData[index];
	newData[index] = ADCread();

	if ((prevData < MIDPOINT) && (newData[index] >= MIDPOINT)) {

		newSlope = newData[index] - prevData;

		if (abs(newSlope - maxSlope[index]) < slopeTolerance) {
			slope[arrayIndex[index]][index] = newSlope;
			timer[arrayIndex[index]][index] = time[index];
			time[index] = 0;

			if (arrayIndex[index] == 0) {
				noMatch[index] = 0;
				arrayIndex[index]++;
			}
			else if ((abs(timer[0] - timer[arrayIndex[index]]) < timerTolerance) && (abs(slope[0][index] - newSlope) < slopeTolerance)) {
				totalTimer = 0;
				for (int i = 0; i < arrayIndex[index]; i++) {
					totalTimer += timer[i][index];
				}
				period[index] = totalTimer;

				timer[0][index] = timer[arrayIndex[index]][index];
				slope[0][index] = slope[arrayIndex[index]][index];
				arrayIndex[index] = 1;
				noMatch[index] = 0;
			}
			else {
				arrayIndex[index]++;
				if (arrayIndex[index] > ARRAY_DEPTH - 1) {
					arrayIndex[index] = 0;
					noMatch[index] = 0;
					maxSlope[index] = 0;
				}
			}
		}
		else if (newSlope > maxSlope[index]) {
			maxSlope[index] = newSlope;
			time[index] = 0;
			noMatch[index] = 0;
			arrayIndex[index] = 0;
		}
		else {
			noMatch[index]++;
			if (noMatch[index] > ARRAY_DEPTH - 1) {
				arrayIndex[index] = 0;
				noMatch[index] = 0;
				maxSlope[index] = 0;
			}
		}
	}

	if (clippingPin > 0)
	{
		if (newData[index] == BOTTOMPOINT || newData[index] == TOPPOINT) {
			digitalWrite(clippingPin, HIGH);
			clipping = true;
		}
	}

	time[index]++;                             // Incremented at sampleRate
	amplitudeTimer[index]++;                   // Incremented at sampleRate

	newMaxAmplitude = abs(MIDPOINT - newData[index]);

	if (newMaxAmplitude > maxAmplitude[index]) {
		maxAmplitude[index] = newMaxAmplitude;
	}

	if (amplitudeTimer[index] >= TIMER_TIMEOUT) {
		amplitudeTimer[index] = 0;
		checkMaxAmp[index] = maxAmplitude[index];
		maxAmplitude[index] = 0;
	}
}

void TC5_Handler(void)
{
	//setMuxPinToRead(currentMuxPin);

	analyzeIncomingData(currentMuxPin);

	//currentMuxPin++;

	TC5->COUNT16.INTFLAG.bit.MC0 = 1;     // Clear interrupt
}

