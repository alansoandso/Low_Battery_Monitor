#include <Arduino.h>
#include <U8g2lib.h>
#include <avr/sleep.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED

// OLED
int sda = A4;
int scl = A5;

// Low Power Monitor
int piezo = 6;
int low_power_warnings; // When this count gets to 5, trigger the alarm

void setup() {
  // put your setup code here, to run once:
  pinMode(piezo, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  u8g2.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  checkPower();
}

void checkPower() {
  long vcc = 0;
  // Implement a count to ignore false alarms
  
  vcc = readVcc();
  displayPower(vcc);

  // Increment count when we have a low reading
  if (vcc < 3400)
    low_power_warnings++;
  else if (low_power_warnings > 0)
    low_power_warnings--;

  // Only when the count reaches 5 should the alarm be raised
  if (low_power_warnings >= 5 )
    soundAlarm();
}

void displayPower(long vcc) {
  char val[5];

  Serial.println(vcc);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setDrawColor(1);
    u8g2.setCursor(0, 16);
    sprintf(val, "%04dmV", vcc);
    u8g2.print(val);
  } while ( u8g2.nextPage() );
}

void soundAlarm() {
  // blank the screen to save power 
  //u8g2.setPowerSave(1);

  for (int i = 0; i < 10 ; i++) {
    tone(piezo, 400, 300);
  }
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
