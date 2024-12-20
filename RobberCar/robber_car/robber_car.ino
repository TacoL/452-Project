/*

  Example of use of the FFT library to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "arduinoFFT.h"

/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 8000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

// Ultrasonic sensor variables
const int trigPin = 12;  
const int echoPin = 13; 
int distanceCounter = 0;

float duration, distance;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

double vRealHistory[20];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03
#define maxAmp 5000

int left_motor = 3;
int right_motor = 5;

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");

  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);

  //Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);

}

double audio_scale(double realAmp){

  double scaleFactor = realAmp / maxAmp;

  return scaleFactor;
  
}

double readDutyCycle(int pin) {
  unsigned long highTime = pulseIn(pin, HIGH); // Time in microseconds the signal is HIGH
  unsigned long lowTime = pulseIn(pin, LOW);  // Time in microseconds the signal is LOW
  unsigned long period = highTime + lowTime;  // Total period of the PWM signal

  if (period == 0) return 0; // Avoid division by zero if no signal is detected

  double dutyCycle = (double)highTime / period; // Calculate duty cycle as a fraction (0-1)
  return dutyCycle;
}

//double[] pidController(){
//  
//}

void loop()
{

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);

  if(distance <= 53){
//    distanceCounter++;
//  }
//
//  if(distanceCounter >= 3){
//    distanceCounter = 0;
//    
////    analogWrite(left_motor, 255*0.1*0.9);
////    analogWrite(right_motor, 255 * 0.9);

    analogWrite(left_motor, 0);
    analogWrite(right_motor, 229.5);
    
  }

  else{
  
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  // ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
  /* Print the results of the sampling according to time */
  // Serial.println("Data:");
  // PrintVector(vReal, samples, SCL_TIME);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  // Serial.println("Weighed data:");
  // PrintVector(vReal, samples, SCL_TIME);
  FFT.compute(FFT_FORWARD); /* Compute FFT */
  // Serial.println("Computed Real values:");
  // PrintVector(vReal, samples, SCL_INDEX);
  // Serial.println("Computed Imaginary values:");
  // PrintVector(vImag, samples, SCL_INDEX);
  FFT.complexToMagnitude(); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");
  // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  // double x = FFT.majorPeak();
  // Serial.println(x, 2); //Print out what frequency is the most dominant.

  // Serial.println("Freq (Hz) : Amplitude");
  // for (int i = 0; i < samples / 2; i++) { // Only first half of bins are meaningful
  //   double frequency = (i * samplingFrequency) / samples;
  //   Serial.print(frequency);
  //   Serial.print(" Hz : ");
  //   Serial.println(vReal[i]); // Amplitude
  // }
  for (int i = 1; i < 20; ++i) {
    vRealHistory[i] = vRealHistory[i - 1];
  }
  vRealHistory[0] = vReal[19];
  
  bool detected = false;
  for (int i = 0; i < 20; ++i) {
    if (vRealHistory[i] > 1000) {
      detected = true;
    }
  }

  double totalvReal = 0;
  for (int i = 0; i < 20; ++i) {
    totalvReal += vRealHistory[i];
  }
  double avg = totalvReal / 20.0;

  if (detected) {
    Serial.print("MIC HEARS BUZZER AT ");
    Serial.println(avg);
    double scaleFactor = audio_scale(avg);
    
//    analogWrite(left_motor, (20*255)/100);
//    analogWrite(right_motor, (30*255)/100);
    double duty_cycle = 0.5 + scaleFactor * 0.4;
    analogWrite(left_motor, 255*0.4*duty_cycle);
    analogWrite(right_motor, 255 * duty_cycle);

//    double leftRead = readDutyCycle(left_motor);
//    double rightRead = readDutyCycle(right_motor);
//
//    Serial.print("left motor pwm = ");
//    Serial.println(leftRead);
//    Serial.print("right motor pwm = ");
//    Serial.println(rightRead);
  }
  else {
    Serial.print("mic doesn't hear buzzer at ");
    Serial.println(avg);
    analogWrite(left_motor, 0);
    analogWrite(right_motor, 0);
    // analogWrite(left_motor, 255*0.1);
    // analogWrite(right_motor, 255*0.5);
  }


  // Serial.print("Sampling Rate: ");
  // Serial.print(samplingFrequency);
  // Serial.println(" Hz");

  // while(1); /* Run Once */
  }
  delay(100); /* Repeat after delay */
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
