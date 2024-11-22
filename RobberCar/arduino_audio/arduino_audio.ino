const int sampleWindow = 50;  // Sample window width in mS (50 mS = 20Hz)
int const AMP_PIN = A0;       // Preamp output pin connected to A0

unsigned int signalLength = 1095;
unsigned int completeSignal[signalLength];
unsigned int * signalIdx = &completeSignal;

void setup()
{
  Serial.begin(9600);
}

unsigned int sampleOnce()
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
  double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
  //Serial.println(volts);
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
    completeSignal[signalIdx] = sampleOnce();
  }
  // If completeSignal is not full, sample until it is
  else {
    while (signalIdx != completeSignal + signalLength - 1) {
      completeSignal[signalIdx] = sampleOnce();
      signalIdx++;
    }
  }
}
void loop()
{
  sampleMic();
}
