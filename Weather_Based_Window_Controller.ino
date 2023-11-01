#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

int Relay = 8;
int RainSensor = 7;
int TempSensor = A3;
int LightSensor = A2;
int ServoPin = 9;
float temperatureC;
int Light;


unsigned long lastServoCheckTime = 0;
const int servoCheckInterval = 2000;

Servo myServo;
LiquidCrystal_I2C lcd(0x3F, 20, 4);

void setup() {
  Serial.begin(9600);
  pinMode(Relay, OUTPUT);
  pinMode(RainSensor, INPUT);
  myServo.attach(ServoPin);

  lcd.init();       // initialize the LCD
  lcd.backlight();  // Turn on the blacklight

  lcd.clear();
  lcd.setCursor(6, 1);
  lcd.print("SYSTEM");
  lcd.setCursor(2, 2);
  lcd.print("INITIALIZATION");
  delay(2000);
}

void loop() {
  if (digitalRead(RainSensor) == LOW) {
    CloseWindow();
    digitalWrite(Relay, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WINDOW STATUS:");
    updateWindowStatus();
    lcd.setCursor(0, 1);
    lcd.print("Temperature: ");
    lcd.print(temperatureC);
    lcd.print(" C");
    lcd.setCursor(0, 2);
    lcd.print("RAIN DETECTED: YES");
    lcd.setCursor(0, 3);
    lcd.print("FAN STATUS: OFF");
    //delay(1000);
  } else if (digitalRead(RainSensor) == HIGH) {
    TempValues();
    if (temperatureC > 50) {
      digitalWrite(Relay, HIGH);
      OpenWindow();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WINDOW STATUS:");
      updateWindowStatus();
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.print(temperatureC);
      lcd.print(" C");
      lcd.setCursor(0, 2);
      lcd.print("RAIN DETECTED: NO");
      lcd.setCursor(0, 3);
      lcd.print("FAN STATUS: ON");
      //delay(1000);
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WINDOW STATUS:");
      updateWindowStatus();
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.print(temperatureC,1);
      lcd.print(" C");
      lcd.setCursor(0, 2);
      lcd.print("RAIN DETECTED: NO");
      lcd.setCursor(0, 3);
      lcd.print("FAN STATUS: OFF");
      digitalWrite(Relay, LOW);
      float LightIntensity = analogRead(LightSensor);

      int servoAngle = map(LightIntensity, 1023, 150, 10, 52);
      // Ensure servo angle is within valid range
      servoAngle = constrain(servoAngle, 5, 50);

      // Control the servo based on LDR values
      myServo.write(servoAngle);
      
      delay(1000);
    }
  }
}

void CloseWindow() {
  myServo.write(5);  // Set the servo angle to 5 degrees
  delay(1000);       // Wait for 1 second (adjust as needed)
  //myServo.detach();  // Detach the servo to stop it
  //while (1)
   // ;  // Stop the program
}
void OpenWindow() {
  myServo.write(50);  // Set the servo angle to 50 degrees
  delay(1000);        // Wait for 1 second (adjust as needed)
  //myServo.detach();   // Detach the servo to stop it
  //while (1)
  //  ;
}
void TempValues() {
  // Get the voltage reading from the LM35
  int reading = analogRead(TempSensor);

  // Convert that reading into voltage
  float voltage = reading * (5000.0 / 1024.0);

  // Convert the voltage into the temperature in Celsius
  temperatureC = voltage / 10.0;
}
void updateWindowStatus() {
  int servoAngle = myServo.read();  
  if (servoAngle > 10) {
    lcd.setCursor(15, 0);
    lcd.print(" OPEN");
  } else {
    lcd.setCursor(14, 0);
    lcd.print("CLOSED");
  }
}