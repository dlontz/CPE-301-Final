//Library includes
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <dht.h>
#include <Stepper.h>


const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 12, 10, 9, 11);
//temperature library
dht DHT;
#define DHT11_PIN 7

//pins
const byte interruptPin = 18;
int motorPin = 8;
const int waterLevelSensorPin = A0;

bool debounceInterruptFlag = false;
unsigned long lastDebounceTime = 0; 
RTC_DS1307 rtc;




// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 53, en = 52, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//State Structure
enum State {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_ERROR,
  STATE_RUNNING
};

// Setting pins 46, 44, 42, and 40 as outputs
void setupPins() {
    // Set PC5 (pin 32), PC3 (pin 34), and PC1 (pin 36) as outputs
    DDRC |= (1 << DDC5) | (1 << DDC3) | (1 << DDC1);
    // Set PD7 (pin 38) as output
    DDRD |= (1 << DDD7);
}

State currentState = STATE_DISABLED;

//Setup
void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  setupPins();
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // The following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time
    // for example to set January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }


  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  DDRE &= ~(1 << DDE4);

  // Enable the internal pull-up resistor on pin 2
  PORTE |= (1 << PORTE4);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptStart, FALLING);
}

void loop() {


	myStepper.setSpeed(10);
	myStepper.step(-stepsPerRevolution);
	delay(1000);
  digitalWrite(motorPin, HIGH);
  lcd.clear();
  int waterLevel = readWaterLevel(); // Read the water level from the sensor
  int chk = DHT.read11(DHT11_PIN);
  switch (currentState) {
    //------------------
    //DISABLED STATE
    //------------------
    case STATE_DISABLED:
    printTemperature();
    printTime();
    turnOffAllLEDs();
    turnOnLEDYellow();
      if (conditionTransitionOnButton()) {
        printTime();
        currentState = STATE_IDLE;
      }
      break;
    //------------------
    //IDLE STATE
    //------------------

    case STATE_IDLE:
    int chk = DHT.read11(DHT11_PIN);
    printTemperature();
    turnOffAllLEDs();
    turnOnLEDGreen();
    Serial.print("Water Level: ");
    Serial.println(waterLevel);
      if (DHT.temperature > 22) {
        printTime();
        currentState = STATE_RUNNING;
      }
      break;
    //------------------
    //RUNNING STATE
    //------------------

    case STATE_RUNNING:
    printTemperature();
    turnOffAllLEDs();
    turnOnLEDBlue();
    Serial.print("Water Level: ");
    Serial.println(waterLevel);
      if (DHT.temperature < 22) {
        printTime();
        currentState = STATE_IDLE;
      }
      break;

    //------------------
    //ERROR STATE
    //------------------
    case STATE_ERROR:
    printTemperature();
    turnOffAllLEDs();
    turnOnLEDRed();
      if (false) {
        printTime();
        currentState = STATE_IDLE;
      }
      break;
  }

}

void printTransition()
{

}

void printTime()
{
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

}

//transition rules; return true = state transition
bool conditionTransitionWaterLevel()
{
  //water level < threshold
  return false;
}

int readWaterLevel() {
    int sensorValue = analogRead(waterLevelSensorPin); // Read the analog value (0 to 1023)
    return sensorValue; // Return the read value
}
//-----------------
//Utility functions
//-----------------
void printSerialMonitor()
{

}

void printLCD()
{
  lcd.print("hello, world!");
}

void printTemperature()
{
  lcd.clear();
  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  lcd.setCursor(0, 0);
  lcd.print("Temperature:");
  lcd.print(DHT.temperature);
  lcd.setCursor(0, 1);
  lcd.print("Humidity:");
  lcd.print(DHT.humidity);
}

void interruptStart() { 
  currentState = STATE_IDLE;
}

void turnOnLEDRed() {
    PORTC |= (1 << PC5); // Turn on the LED on pin 32
}

void turnOnLEDGreen() {
    PORTC |= (1 << PC3); // Turn on the LED on pin 34
}

void turnOnLEDYellow() {
    PORTC |= (1 << PC1); // Turn on the LED on pin 36
}

void turnOnLEDBlue() {
    PORTD |= (1 << PD7); // Turn on the LED on pin 38
}
void turnOffAllLEDs() {
    PORTC &= ~((1 << PC5) | (1 << PC3) | (1 << PC1));
    PORTD &= ~(1 << PD7);
}