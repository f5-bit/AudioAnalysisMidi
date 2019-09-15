/*
  Name:		AudioAnalysis.ino
  Created:	06.06.2019 23:14:24
  Author:	Franca Bittner
*/

#include "src/AudioFrequencyMeterMultiplexed.h"
#include <MIDIUSB.h>
#include <frequencyToNote.h>
#include <pitchToNote.h>

#define DEPTH         60                      // Defines depth of the array for averaged frequencies
#define HUMAN_RATE    50                      // Defines 50ms corresponding to 20 notes/s
#define MAX_DURATION  1000                    // Defines the max play duration of the note

int s0 = 2;
int s1 = 3;
int s2 = 4;
AudioFrequencyMeterMultiplexed meter;

int count = 0;   //which y pin we are selecting
int input = A0;


int notesArray[DEPTH];                        // Array to store detected notes and find the "correct" note which occurred the most often
int occurrences[DEPTH];                       // Array in which the number of occurrences for each note are stored
bool marked[DEPTH];                           // Array to indicate which of the notes have been checked
int frequencyIndex = 0;                       // Used to navigate to where the current note must be stored

int previousNote;
unsigned int startTime;                       // Used to determine when the note must stop
unsigned int humanTime;                       // Used to determine when the next note can be sampled (using HUMAN_RATE timing)

int intensity = 64;                           // The volume of the played note is fixed at 64


void setup() {
	Serial.begin(115200);
	Serial.println("started");

	meter.setBandwidth(70.00, 1500);    // Ignore frequency out of this range
	meter.begin(A0, 45000);             // Intialize A0 at sample rate of 45kHz
}


// TODO: Find out the best solution: either loop the multiplexer (set mux control pins) and get frequency or
//       get frequency (set mux control pins there) and loop through all pins
// --> loop through multiplexer first and then get frequency for each pin. Otherwise input is incorrect!
void loop() {

	int frequency = meter.getFrequencyMux(0);
	Serial.print(frequency);
	Serial.println(" Hz");

	if (frequency > 0)
	{
		int noteIndex = searchForNote(frequency); // Find the index of the corresponding frequency
		int note = notePitch[noteIndex];          // Use that index to find the corresponding note in the LUT
		notesArray[frequencyIndex++] = note;      // Store the note and continue to next value in array

		if (frequencyIndex > DEPTH)               // If all the notes have been stored
		{
			frequencyIndex = 0;                     // Reset the index
			int i, j;

			/*Reset all the occurences and marked positions*/
			for (i = 0; i < DEPTH; i++)
			{
				occurrences[i] = 0;
				marked[i] = 0;
			}

			/*Count the number of occurrences*/
			for (i = 0; i < DEPTH; i++)
			{
				for (j = 0; j < DEPTH; j++)
				{
					// If notes are the same and the note has not been marked yet
					if ((!marked[j]) && (notesArray[j] == notesArray[i]))
					{
						occurrences[i]++;                 // Increment the number of occurrences
						marked[j] = true;                 // Signal the note as marked
					}
				}
			}

			int numberOfdifferentFrequencies = 0;   // Used to determine how many different Frequencies have been detected

			for (i = 0; i < DEPTH; i++)
			{
				// If the counter does not equal zero
				if (occurrences[i] != 0)
				{
					// Store the the various detected Frequencies
					notesArray[numberOfdifferentFrequencies] = notesArray[i];
					// And the number of occurrences for each note
					occurrences[numberOfdifferentFrequencies] = occurrences[i];
					numberOfdifferentFrequencies++;      // Increment the number of detected Frequencies
				}
			}

			/*Search for the maximum number of occurrences to discriminate the played note*/
			int maxNumberOfFrequencies = occurrences[0];
			int rightIndex = 0;

			for (i = 0; i < numberOfdifferentFrequencies; i++);
			{
				// If a new maximum exist
				if (occurrences[i] > maxNumberOfFrequencies)
				{
					// Update the value
					maxNumberOfFrequencies = occurrences[i];
					// Update the index
					rightIndex = i;
				}
			}
			note = notesArray[rightIndex];          // Note to be played is that with the most occurrences
			// If the specified time has elapsed before the next note
			if (millis() - humanTime > HUMAN_RATE)
			{
				humanTime = millis();                 // Update the timer
				startTime = millis();                 // Update the note duration
				noteOff(0, previousNote, intensity);  // Stop playing the previous note
				previousNote = note;                  // Update previous note with the new one
				Serial.println(note);                 // Print the note to be played
				noteOn(0, note, intensity);           // Play the note!
			}
		}
	}

	if (millis() - startTime > MAX_DURATION)    // If maximum time elapsed
		noteOff(0, previousNote, intensity);      // Turn the note off
}

int searchForNote(float frequency)
{
	float minimum = abs(frequency - noteFrequency[0]);
	float newMinimum;
	int index = 0;

	/*Search for the nearest frequency that is in the vector*/
	for (int i = 0; i < NUMBER_OF_NOTES - 1; i++)
	{
		newMinimum = abs(frequency - noteFrequency[i]);
		if (newMinimum < minimum)
		{
			minimum = newMinimum;
			index = i;
		}
	}

	return index;
}

void noteOn(byte channel, byte pitch, byte velocity) {
	midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
	MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
	midiEventPacket_t noteOff = { 0x08, 0x80 | channel, pitch, velocity };
	MidiUSB.sendMIDI(noteOff);
}
