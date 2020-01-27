#include <Arduino.h>
// the setup function runs once when you press reset or power the board

#include <LiquidCrystal.h>
#include <SoftwareSerial.h>//allows to use TX and RX communication on the Arduino's other pins rather than only using the default TX and RX pins
#include <GSM.h>
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7,8,9,10,11,12);
SoftwareSerial wifi_module(2,3); // Connect TX pin of esp to the pin 2 of Arduino and RX pin of esp to the pin 3 of Arduino
int red_led_pin = D6;
int green_led_pin = D7;
int buzzer_pin = D2;
int gas_sensor_pin = A0;
int potPin1 = 2;



WiFiClient espClient;
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "TOSIT4uDQC2S52JrezyA4On0rzfObpnr";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "YourNetworkName";
char pass[] = "YourPassword";

BlynkTimer timer;

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, millis() / 1000);
}

void setup() {
 
  Serial.begin(115200);
  pinMode(red_led_pin ,OUTPUT);
  pinMode(green_led_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(gas_sensor_pin, INPUT);
  pinMode(potPin1, INPUT);
   int analogSensor= analogRead(gas_sensor_pin);
esp8266_command("AT+RST\r\n",2000,DEBUG); // reset module
  lcd.begin(16, 2);
lcd.clear(); 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("IP address: " + WiFi.localIP().toString()); 
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent); 
}


// the loop function runs over and over again forever
void loop() {
  client.loop();
  lcd.setCursor(0,0); // Sets the cursor to col 0 and row 0
lcd.print("SensorVal1: "); // Prints Sensor Val: to LCD
lcd.print(analogRead(potPin1)); // Prints value on Potpin1 to LCD

      int analogSensor = analogRead(gas_sensor_pin);
      lcd.setCursor(0,0); // Sets the cursor to col 0 and row 0
lcd.print("SensorVal1: "); // Prints Sensor Val: to LCD
lcd.print(analogRead(potPin1)); // Prints value on Potpin1 to LCD
    if (analogSensor > 400)
 {
  digitalWrite(red_led_pin, HIGH);
  digitalWrite(green_led_pin, LOW);
  tone(buzzer_pin, 1000, 200);
 }
  else{
      digitalWrite(red_led_pin,LOW);
      digitalWrite(green_led_pin,HIGH);
      noTone(buzzer_pin);
  }
  Blynk.run();
  timer.run(); 
  
}