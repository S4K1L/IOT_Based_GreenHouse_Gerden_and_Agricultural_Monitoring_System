  //Include the library files
#include<SPI.h>
#include<LCD.h>
#include<Wire.h>
#include <LiquidCrystal_I2C.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>

//Initialize the LCD display
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7,3,POSITIVE);//instance for lcd display
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);//instance for lcd display
//LiquidCrystal_I2C lcd(0x27, 16, 2);


char auth[] = "2D3PrAdhSbqRusxwC0wvs0WWgFKo-bys";  //Enter your Blynk Auth token
char ssid[] = "cse-lab";  //Enter your WIFI SSID
char pass[] = "Cselab123";  //Enter your WIFI Password

DHT dht(D4, DHT11);//(DHT sensor pin,sensor type)  D4 DHT11 Temperature Sensor
BlynkTimer timer;

//Define component pins
#define soil A0     //A0 Soil Moisture Sensor
#define LDR D7      //D7 LDR Motion Sensor

#define RELAY_PIN_1 D6   //D6 motor
#define RELAY_PIN_2 D8   //D8 Fan
#define RELAY_PIN_3 D5   //D5 Light

#define VPIN_BUTTON_1    V12 //Motor
#define VPIN_BUTTON_2    V13 //Fan
#define VPIN_BUTTON_3    V11 //Light 

//Create three variables for pressure
double T, P;
char status;
int LDR_ToggleValue;
int flag=0;

const int Buzzer = 16; //D0 Buzzer

void checkPhysicalButton();
int relay1State = LOW;
int relay2State = LOW;
int relay3State = LOW;

void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.backlight();
  pinMode(LDR, INPUT);

 pinMode(RELAY_PIN_1, OUTPUT);
 pinMode(RELAY_PIN_2, OUTPUT);
 pinMode(RELAY_PIN_3, OUTPUT);
 pinMode(Buzzer, OUTPUT);

//During Starting all Relays should TURN OFF
 digitalWrite(RELAY_PIN_1, HIGH);
 digitalWrite(RELAY_PIN_2, HIGH);
 digitalWrite(RELAY_PIN_3, HIGH);

  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  dht.begin();

  Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
  Blynk.virtualWrite(VPIN_BUTTON_2, relay2State);
  Blynk.virtualWrite(VPIN_BUTTON_3, relay3State);

  lcd.setCursor(0, 0);
  lcd.print(" SMUCT Pirate's ");
  for (int a = 5; a <= 10; a++) {
    lcd.setCursor(a, 1);
    lcd.print(".");
    delay(500);
  }
  lcd.clear();
  lcd.setCursor(11, 1);
  lcd.print("W:OFF");
  //Call the function
  timer.setInterval(100L, soilMoistureSensor);
  timer.setInterval(100L, DHT11sensor);
  timer.setInterval(100L, LDRsensor);
}

BLYNK_WRITE(V6)
{
 LDR_ToggleValue = param.asInt();  
}

BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
  Blynk.syncVirtual(VPIN_BUTTON_3);
}

// When App button is pushed - switch the state
BLYNK_WRITE(VPIN_BUTTON_1) {
  relay1State = param.asInt();
  if(relay1State == 1){
  digitalWrite(RELAY_PIN_1, LOW);
  }
  else{
    digitalWrite(RELAY_PIN_1, HIGH);
  }
}

BLYNK_WRITE(VPIN_BUTTON_2) {
  relay2State = param.asInt();
  if(relay2State == 1){
  digitalWrite(RELAY_PIN_2, LOW);
  }
  else{
    digitalWrite(RELAY_PIN_2, HIGH);
  }
}

BLYNK_WRITE(VPIN_BUTTON_3) {
  relay3State = param.asInt();
  if(relay3State == 1){
  digitalWrite(RELAY_PIN_3, LOW);
  }
  else{
    digitalWrite(RELAY_PIN_3, HIGH);
  }
}

//Get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t);

  lcd.setCursor(12, 0);
  lcd.print("H:");
  lcd.print(h);

}


//Get the soil moisture values
void soilMoistureSensor() {
  int value = analogRead(soil);
  value = map(value, 0, 1024, 0, 100);
  value = (value - 100) * -1;

  Blynk.virtualWrite(V3, value);
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(value);
  lcd.print(" ");

}

  //Get the Laser Security
void LDRsensor ()
{
  int isButtonPressed = digitalRead(LDR);
  if (isButtonPressed==1 && flag==0) {
    Blynk.logEvent("ldrwarning","WARNNG! Thief in the Green House!");
    WidgetLED LED(V5);
    LED.on();
        digitalWrite(Buzzer, HIGH);
        delay(200);
        digitalWrite(Buzzer, HIGH);
        delay(200);
        digitalWrite(Buzzer, HIGH);
        delay(200);
        digitalWrite(Buzzer, LOW);
    flag=1;
  }
  else if (isButtonPressed==0)
  {
    flag=0;
    WidgetLED LED(V5);
    LED.off();
  }
}


void loop() {
    if (LDR_ToggleValue == 1)
    {
    lcd.setCursor(8, 0);
    lcd.print("L:1 ");
      LDRsensor();
      }
     else
     {
    lcd.setCursor(8, 0);
    lcd.print("L:0");
    WidgetLED LED(V5);
    LED.off();
     }

if (relay1State == HIGH)
{
  lcd.setCursor(11, 1);
  lcd.print("W:ON ");
  }
  else if (relay1State == LOW)
  {
    lcd.setCursor(11, 1);
    lcd.print("W:OFF");
  }

if (relay2State == HIGH)
{
  lcd.setCursor(5, 1);
  lcd.print("F:ON ");
  }
  else if (relay2State == LOW)
  {
    lcd.setCursor(5, 1);
    lcd.print("F:OFF");
  }  
      
  Blynk.run();//Run the Blynk library
  timer.run();//Run the Blynk timer

  }
