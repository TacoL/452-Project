#include "bandpass_iir.h"
const double sampleWindow = 0.16666666666;  // 6000 Hz
int const AMP_PIN = A0;       // Preamp output pin connected to A0

const int signalLength = 100;
double completeSignal[signalLength];
double * signalIdx = completeSignal;

const int filteredSignalLength = 100;
double filteredSignal[filteredSignalLength];

void setup()
{
  Serial.begin(9600);
}

double sampleOnce()
{
  unsigned int sample = 0;
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS and then plot data
  unsigned long startMillis = millis(); // Start of sample window
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(AMP_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  //Serial.println(peakToPeak);
  double volts = (peakToPeak * 5.0) / 1024.0;  // convert to volts
  //Serial.println(volts);
  if (volts > 5) {
    volts = 0;
  }
  return volts;
}

void sampleMic()
{
  // If completeSignal is full, delete first entry and add new one
  if (signalIdx == completeSignal + signalLength - 1)
  {
    for (int i = 0; i < signalLength - 1; ++i)
    {
      completeSignal[i] = completeSignal[i+1]; // shifts all values left
    }

    // Sample once
    *signalIdx = sampleOnce();
  }
  // If completeSignal is not full, sample until it is
  else {
    while (signalIdx != completeSignal + signalLength - 1) {
      *signalIdx = sampleOnce();
      signalIdx++;
    }
  }
}

// void convolve(double input[], double filter[])
// {
//   // Reset filtered signal
//   for (int i = 0; i < filteredSignalLength; ++i)
//   {
//     filteredSignal[i] = 0;
//   }

//   // Perform the convolution
//   for (int n = 0; n < filteredSignalLength; ++n) {
//     for (int k = 0; k < BPL; ++k) {
//       if (n - k >= 0 && n - k < signalLength) {
//         filteredSignal[n] += input[n - k] * filter[k];  // Convolution sum
//       }
//     }
//   }
// }

// Function to apply the IIR filter (Direct-Form II)
void filter_signal() {

  // Reset filtered signal
  for (int i = 0; i < filteredSignalLength; ++i)
  {
    filteredSignal[i] = 0;
  }

  double w[MWSPT_NSEC][2] = {0}; // Temporary buffers for each section's state (previous inputs and outputs)
  double z[MWSPT_NSEC][2] = {0}; // State buffers for previous inputs (x) and outputs (y)

  // Loop through each input sample
  for (int n = 0; n < signalLength; n++) {
    // Initialize the input value for the current sample
    double input = completeSignal[n];

    // Process each section
    for (int i = 0; i < MWSPT_NSEC; i++) {
      // Apply the biquad difference equation: y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] - a1 * y[n-1] - a2 * y[n-2]

      // Compute the output for this section
      double output = BP_NUM[i][0] * input + BP_NUM[i][1] * z[i][0] + BP_NUM[i][2] * z[i][1] 
                       - BP_DEN[i][1] * w[i][0] - BP_DEN[i][2] * w[i][1];

      // Store the current input and output into the state buffers for the next iteration
      z[i][1] = z[i][0]; // Shift previous input
      z[i][0] = input;   // Update the input for this section
      w[i][1] = w[i][0]; // Shift previous output
      w[i][0] = output;  // Update the output for this section

      // Set the current input for the next section to be the output of the current section
      input = output;
    }

    // The final output after passing through all sections
    filteredSignal[n] = input;
  }
}

double getPWM() {
  // filter the signal with the bandpass

  // read filtered signal and return appropriate PWM
}

void loop()
{
  // Serial.println(sampleOnce());
  sampleMic();

  // for (int i = 0; i < signalLength; ++i) {
  //   Serial.println(completeSignal[i]);
  // }

  filter_signal();
  Serial.println("filtered signal");

  for (int i = 0; i < filteredSignalLength; ++i) {
    Serial.println(filteredSignal[i]);
  }

  while (true)
  {

  }
}
