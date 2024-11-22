#include <arduinoFFT.h>

// ArduinoFFT<double> FFT = ArduinoFFT<double>(samp_arr, imag, numSamples, samplingRate);


const uint16_t numSamples = 128; //This value MUST ALWAYS be a power of 2
const double samplingRate = 8928;

double samp_arr[numSamples];
double imag[numSamples] = {0.0};

ArduinoFFT<double> FFT = ArduinoFFT<double>(samp_arr, imag, numSamples, samplingRate);

// const int sampleWindow = 50;  // Sample window width in mS (50 mS = 20Hz)
int const AMP_PIN = A0;       // Preamp output pin connected to A0
// unsigned int sample;

// const int numSamples = 1000; // Number of samples to measure
// unsigned long startTime = micros(); // Start time in microseconds

void setup()
{
  Serial.begin(9600);
  // cli(); // Disable interrupts
  // ADCSRA = (ADCSRA & 0xF8) | 0x02; // Set prescaler to 16 (77 kHz max ADC rate)
  // sei(); // Enable interrupts
}

void loop()
{
  
  // for (int j = 0; j < 1000; j++) {
  //   imag[j] = 0;
  // }
  
  // Serial.println(imag[93]);

  // unsigned long startMillis = millis(); // Start of sample window
  // unsigned int peakToPeak = 0;   // peak-to-peak level

  // unsigned int signalMax = 0;
  // unsigned int signalMin = 1024;

  // int sample = analogRead(AMP_PIN);
  // Serial.println(sample);

  // collect data for 50 mS and then plot data
  // while (millis() - startMillis < sampleWindow)
  // {
  //   sample = analogRead(AMP_PIN);
  //   if (sample < 1024)  // toss out spurious readings
  //   {
  //     if (sample > signalMax)
  //     {
  //       signalMax = sample;  // save just the max levels
  //     }
  //     else if (sample < signalMin)
  //     {
  //       signalMin = sample;  // save just the min levels
  //     }
  //   }
  // }
  // peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  // Serial.println(peakToPeak);
  //double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
  //Serial.println(volts);

  // const int numSamples = 1000; // Number of samples to measure
  unsigned long startTime = micros(); // Start time in microseconds
  // float samp_arr[1000];

  for (int i =0; i < numSamples; i++) {
    float q = analogRead(A0); // Read from analog pin
    samp_arr[i] = q;
    // Serial.println(q);
  }

  unsigned long elapsedTime = micros() - startTime; // Time elapsed
  float samplingRate = (float)numSamples / (elapsedTime / 1e6); // Samples per second
  // Serial.println(samplingRate);

  //  FFT.Windowing(samp_arr, numSamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Apply windowing
  FFT.compute(samp_arr, imag, numSamples, FFT_FORWARD);                // Perform FFT
  FFT.complexToMagnitude(samp_arr, imag, numSamples);                  // Compute magnitudes
  // Serial.println(samp_arr[5]);

  // Print the frequency bins and corresponding magnitudes
  Serial.println("Freq (Hz) : Amplitude");
  for (int i = 0; i < numSamples / 2; i++) { // Only first half of bins are meaningful
    double frequency = (i * samplingRate) / numSamples;
    Serial.print(frequency);
    Serial.print(" Hz : ");
    Serial.println(samp_arr[i]); // Amplitude
  }


  Serial.print("Sampling Rate: ");
  Serial.print(samplingRate);
  Serial.println(" Hz");

  // for (int i = 0; i < numSamples; i++) {
  //   samp_arr[i] = 0;
  // }
  // for (int i = 0; i < numSamples; i++) {
  //   imag[i] = 0;
  // }



  delay(30000); // Wait for 1 second before measuring again
}
