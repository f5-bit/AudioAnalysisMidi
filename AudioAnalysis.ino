/*
  Name:		AudioAnalysis.ino
  Created:	06.06.2019 23:14:24
  Author:	Franca Bittner
*/

#include "src/AudioFrequencyMeterMultiplexed.h"

int s0 = 2;
int s1 = 3;
int s2 = 4;
AudioFrequencyMeterMultiplexed meter(s0, s1, s2);

int count = 0;   //which y pin we are selecting
int input = A0;

void setup() {
//  pinMode(s0, OUTPUT);
//  pinMode(s1, OUTPUT);
//  pinMode(s2, OUTPUT);

  Serial.begin(115200);
  Serial.println("started");

  meter.setBandwidth(70.00, 1500);    // Ignore frequency out of this range
  meter.begin(A0, 45000);             // Intialize A0 at sample rate of 45kHz
}


// TODO: Find out the best solution: either loop the multiplexer (set mux control pins) and get frequency or
//       get frequency (set mux control pins there) and loop through all pins
// --> loop through multiplexer first and then get frequency for each pin. Otherwise input is incorrect!
void loop() {

  //for (count = 0; count <= 7; count++) {

  //  //// select the bit
  //  //r0 = bitRead(count, 0);   // use this with arduino 0013 (and newer versions)
  //  //r1 = bitRead(count, 1);   // use this with arduino 0013 (and newer versions)
  //  //r2 = bitRead(count, 2);   // use this with arduino 0013 (and newer versions)

  //  ////r0 = count & 0x01;      // old version of setting the bits
  //  ////r1 = (count>>1) & 0x01;      // old version of setting the bits
  //  ////r2 = (count>>2) & 0x01;      // old version of setting the bits

  //  //digitalWrite(s0, r0);
  //  //digitalWrite(s1, r1);
  //  //digitalWrite(s2, r2);

  //  ////Either read or write the multiplexed pin here
  //  //if (count == 1) {
  //  //	//Serial.println(analogRead(A4));
  //  float frequency = meter.getFrequencyMux(count);

  //    Serial.print(count);
  //    Serial.print(frequency);
  //    Serial.println(" Hz");
  //  //}
  //}

	float frequency = meter.getFrequency();
	
		Serial.print(frequency);
		Serial.println(" Hz");
	
}
