int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int reactionTimen = 1000; // time between reads of potentiometr.

unsigned int minFrequency = 10000; //min frequency (31 Hz is real min)
unsigned int maxFrequency = 65535; //max frequency (65535 Hz is real maximum)
unsigned int maxInput = 1023;
unsigned int frequency;

bool isStarted = false;

void setup() {
  Serial.begin(9600); 
  frequency =0;
}

void loop() {  
  
  sensorValue = analogRead(sensorPin);
  unsigned int newFrequency = GetFrequency(sensorValue);
  if (newFrequency != frequency)
  {
    if(frequency >0)
      noTone(ledPin); 
    frequency = newFrequency;
    Serial.print(sensorValue);
    Serial.print(" got from sensor; ");    
    tone(ledPin, frequency);
    Serial.print(frequency);
    Serial.println(" Hz");
  }  
  delay(reactionTimen);  
}

unsigned int GetFrequency(int sensorInput)
{
  return (unsigned int)((float)(maxFrequency - minFrequency)/maxInput * sensorInput) + minFrequency;
}
