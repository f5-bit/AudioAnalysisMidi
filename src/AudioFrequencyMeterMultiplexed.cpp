#include "AudioFrequencyMeterMultiplexed.h"

#define ARRAY_DEPTH             20
#define NOT_INITIALIZED         -1
#define TIMER_TIMEOUT           1000
#define BOTTOMPOINT             0
#define MIDPOINT                127
#define TOPPOINT                255

static int slopeTolerance;
static int timerTolerance;

static bool clipping;
static int clippingPin;

static int  newData[8], prevData;                           // Variables to store ADC result

static unsigned int time, totalTimer;                    // Variables used to compute period
static volatile unsigned int period;

static int arrayIndex;                                   // Index to save data in the correct position of the arrays
static int timer[ARRAY_DEPTH];                           // Array to store trigger events
static int slope[ARRAY_DEPTH];                           // Array to store changing in slope events

static int maxSlope;                                     // Variable to store max detected amplitude
static int newSlope;                                     // Variable to store a new slope

static int noMatch;                                      // Variable to store non-matching trigger events

static unsigned int amplitudeTimer;                      // Variable to reset trigger
static int maxAmplitude;                                 // Variable to store the max detected amplitude
static int newMaxAmplitude;                              // Variable used to check if maxAmplitude must be updated

static volatile int checkMaxAmp;                         // Used to update the new frequency in base of the AMplitude threshold

// For multiplexer

int r0 = 0;      //value of select pin at the 4051 (s0)
int r1 = 0;      //value of select pin at the 4051 (s1)
int r2 = 0;      //value of select pin at the 4051 (s2)


AudioFrequencyMeterMultiplexed::AudioFrequencyMeterMultiplexed(int s0, int s1, int s2) {
	_s0 = s0;
	_s1 = s1;
	_s2 = s2;
	initializeVariables();
}

float AudioFrequencyMeterMultiplexed::getFrequencyMux(int muxPin)
{
	r0 = bitRead(muxPin, 0);   // use this with arduino 0013 (and newer versions)
	r1 = bitRead(muxPin, 1);   // use this with arduino 0013 (and newer versions)
	r2 = bitRead(muxPin, 2);   // use this with arduino 0013 (and newer versions)

	digitalWrite(_s0, r0);
	digitalWrite(_s1, r1);
	digitalWrite(_s2, r2);

	//Either read or write the multiplexed pin here

	return getFrequency();
}

void AudioFrequencyMeterMultiplexed::begin(int pin, unsigned int rate)
{
	samplePin = pin;                              // Store ADC channel to sample
	sampleRate = rate;                            // Store sample rate value
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

float AudioFrequencyMeterMultiplexed::getFrequency()
{
	float frequency = -1;

	if (checkMaxAmp > amplitudeThreshold) {
		frequency = (float)(sampleRate / period);

		if ((frequency < minFrequency) || (frequency > maxFrequency)) {
			frequency = -1;
		}
	}

	return frequency;
}

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

	for (int i = 0; i < sizeof(newData); i++)
	{
		newData[i] = 0;
	}

	prevData = MIDPOINT;
	time = 0;
	arrayIndex = 0;
	maxSlope = 0;
	noMatch = 0;
	amplitudeTimer = 0;
	maxAmplitude = 0;
	checkMaxAmp = 0;
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

void TC5_Handler(void)
{
	analyzeIncomingData(0);

	TC5->COUNT16.INTFLAG.bit.MC0 = 1;     // Clear interrupt
}

void analyzeIncomingData(int index)
{
	prevData = newData[index];
	newData[index] = ADCread();

	if ((prevData < MIDPOINT) && (newData[index] >= MIDPOINT)) {

		newSlope = newData[index] - prevData;

		if (abs(newSlope - maxSlope) < slopeTolerance) {
			slope[arrayIndex] = newSlope;
			timer[arrayIndex] = time;
			time = 0;

			if (arrayIndex == 0) {
				noMatch = 0;
				arrayIndex++;
			}
			else if ((abs(timer[0] - timer[arrayIndex]) < timerTolerance) && (abs(slope[0] - newSlope) < slopeTolerance)) {
				totalTimer = 0;
				for (int i = 0; i < arrayIndex; i++) {
					totalTimer += timer[i];
				}
				period = totalTimer;

				timer[0] = timer[arrayIndex];
				slope[0] = slope[arrayIndex];
				arrayIndex = 1;
				noMatch = 0;
			}
			else {
				arrayIndex++;
				if (arrayIndex > ARRAY_DEPTH - 1) {
					arrayIndex = 0;
					noMatch = 0;
					maxSlope = 0;
				}
			}
		}
		else if (newSlope > maxSlope) {
			maxSlope = newSlope;
			time = 0;
			noMatch = 0;
			arrayIndex = 0;
		}
		else {
			noMatch++;
			if (noMatch > ARRAY_DEPTH - 1) {
				arrayIndex = 0;
				noMatch = 0;
				maxSlope = 0;
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

	time++;                             // Incremented at sampleRate
	amplitudeTimer++;                   // Incremented at sampleRate

	newMaxAmplitude = abs(MIDPOINT - newData[index]);

	if (newMaxAmplitude > maxAmplitude) {
		maxAmplitude = newMaxAmplitude;
	}

	if (amplitudeTimer >= TIMER_TIMEOUT) {
		amplitudeTimer = 0;
		checkMaxAmp = maxAmplitude;
		maxAmplitude = 0;
	}
}