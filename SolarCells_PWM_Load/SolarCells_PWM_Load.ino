/*
  Fade

  This example shows how to fade an LED on pin 9 using the analogWrite()
  function.

  The analogWrite() function uses PWM, so if you want to change the pin you're
  using, be sure to use another PWM capable pin. On most Arduino, the PWM pins
  are identified with a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Fade
*/

int led = 13;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 1;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);

  //First clear all three prescaler bits:
int prescalerVal = 0x07; //create a variable called prescalerVal and set it equal to the binary                                                       number "00000111"
TCCR0B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"

//Now set the appropriate prescaler bits:
 prescalerVal = 1; //set prescalerVal equal to binary number "00000001"
TCCR0B |= prescalerVal; //OR the value in TCCR0B with binary number "00000001"
}

// the loop routine runs over and over again forever:
void loop() {

  analogWrite(led, 0);
  delay(100);
  //for(int i=45; i<= 140; i++)
  for(int i=0; i<= 250; i++)
  {
    analogWrite(led, i);
    delay(30);
  }
  delay(100);
  // set the brightness of pin 9:
  //analogWrite(led, brightness);

  // change the brightness for next time through the loop:
  //brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  //if (brightness <= 0 || brightness >= 255) {
  //  fadeAmount = -fadeAmount;
  //}
  // wait for 30 milliseconds to see the dimming effect
  //delay(5);
  
}


