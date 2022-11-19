#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <GyverButton.h>
#include <avr/eeprom.h>
#include "DHT.h"

// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain



//#define DHTPIN 2     // what digital pin we're connected to
const int tSensor = 2;
#define RelayPin 10
#define buttonPinInc 3
#define buttonPinDec 6
int deltaTemp = 3;
const int MIN_TEMP = -10;
const int  MAX_TEMP = 40;

bool relayState = 0;
int goalTemp = 0;
int curTemp = 0;

//#define DHTTYPE DHT21   // DHT 22  (AM2302), AM2321

//DHT dht(DHTPIN, DHTTYPE);
OneWire ds(tSensor);
LiquidCrystal_I2C lcd(0x27, 16, 2);
GButton but1(buttonPinInc);
GButton but2(buttonPinDec);

int getTemp() {
  int t = 0;
  byte data [2];
  ds.reset();
  ds.write(0xCC);
  ds.write(0x44);
  delay(750);
  ds.reset();
  ds.write(0xCC);
  ds.write(0xBE);
  data[0] = ds.read();
  data[1] = ds.read();
  t = (data[1] << 8) + data[0];
  t = t >> 4;
  return t;
  //Serial.println(Temp);
}
void setup() {
  // 2 - dht
  // 4,7 - gnd
  
  // 10 - relay pin
  pinMode(4, OUTPUT);
  
  pinMode(RelayPin, OUTPUT);
  digitalWrite(4, LOW);
  
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  
  Serial.begin(9600);
  //dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("T:     *C R:   "));
  lcd.setCursor(0, 1);
  lcd.print(F("H:     % G:   "));
  Serial.begin(9600);
  goalTemp = eeprom_read_word(5);
}

void proc()
{
  if (goalTemp < MIN_TEMP+1)
  { lcd.setCursor(13, 1);
    lcd.print(F("   "));
    lcd.setCursor(13, 1);
    lcd.print(F("---"));
    relayState = 0;
    return;
  }
  if (goalTemp > MAX_TEMP - 1)
  {
    lcd.setCursor(13, 1);
    lcd.print(F("   "));
    lcd.setCursor(13, 1);
    lcd.print(F("---"));
    relayState = 1;
    return;
  }
  termControl();
}

void termControl()
{
  if (curTemp < goalTemp - deltaTemp)
  {
    relayState = 1;
  }
  else if (curTemp > goalTemp)
    relayState = 0;
}

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;
int h,t;
int prevGoalTemp = goalTemp;
void loop() {
  
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
  //  h = dht.readHumidity();
    // Read temperature as Celsius (the default)
  //  t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
     t = getTemp();

    if (isnan(h) || isnan(t)) {
      Serial.println("isnan");
      return;
    }
    Serial.print(t);
  }

  //Serial.println(h);
  lcd.setCursor(3, 0);
  lcd.print(F("  "));
  lcd.setCursor(3, 0);
  lcd.print(t);
  lcd.setCursor(3, 1);
  lcd.print(F("  "));
  lcd.setCursor(3, 1);
  lcd.print(h);
  lcd.setCursor(13, 0);
  lcd.print(F("   "));
  lcd.setCursor(13, 0);
  if (relayState)lcd.print(F("On"));
  else lcd.print(F("Off"));
  lcd.setCursor(13, 1);
  lcd.print(F("   "));
  lcd.setCursor(13, 1);
  lcd.print(goalTemp);
  Serial.print(" ");
  Serial.println(goalTemp);

  but1.tick();
  
  if (but1.isClick()) {
    if (goalTemp + 1 >= MAX_TEMP) goalTemp = MAX_TEMP;
    else goalTemp++;
  }
  if (but1.isStep()) {
    if (goalTemp + 5 >= MAX_TEMP) goalTemp = MAX_TEMP;
    else goalTemp += 5;
  }
  but2.tick();
  if (but2.isClick()) {
    if (goalTemp - 1 <= MIN_TEMP) goalTemp = MIN_TEMP;
    else goalTemp--;
  }
  if (but2.isStep()) {
    if (goalTemp - 5 <= MIN_TEMP) goalTemp = MIN_TEMP;
    else goalTemp -= 5;
  }

  if(goalTemp != prevGoalTemp){
    prevGoalTemp = goalTemp;
    eeprom_update_word(5, goalTemp);
  }
  curTemp = t;
  proc();
  digitalWrite(RelayPin, !relayState);

}
