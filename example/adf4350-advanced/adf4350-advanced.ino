#include <SPI.h>
#include "ADF4350.h"

#if defined(ESP8266)
#define PLL_LE_PIN D8
#else
#define PLL_LE_PIN 8
#endif

#define SCAN_I2C

ADF4350 PLL(PLL_LE_PIN);

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//i2c address and pin config for pcf8574 board, maybe differ
// DB2OO changed
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);
// LiquidCrystal_I2C lcd(0x20, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);

#include <Encoder.h>
Encoder encoder(3, 2); //pins with interrupt support prefer
long oldPosition  = 0;
#define ENCODER_SW_PIN 5
#define ENCODER_CLICK_STEP 4.0 //steps between fixed position of encoder

#if defined(ESP8266)
#define LOCK_PIN D5 //lock status from ADF4350 via MUXOUT pin
#else
#define LOCK_PIN 4 //lock status from ADF4350 via MUXOUT pin
#endif

// DB2OO, 29.12.19: don't define HEAT_PIN, as I don't have one ;-)
//#define HEAT_PIN 6 //heat status from FE-5680A rubidium frequency standard
bool pll_lock = false;
bool rubid_heating = false;

#if defined(ESP8266)
#define eeprom_read_dword(x) (0)
#define eeprom_read_byte(x) (0)
#define eeprom_write_dword(x,y) (0)
#define eeprom_write_byte(x,y) (0)
#else

#include <avr/eeprom.h>

#endif
#define EEPROM_WRITE_TIMEOUT 10000UL //save current config to EEPROM after 10 sec of last change, DB2OO
#define EEPROM_ADDR_FREQ 0
#define EEPROM_ADDR_CURPOS 5
#define EEPROM_ADDR_POWER 6
unsigned long eeprom_write_timeout = 0;

#ifdef _ADF4351
//DB2OO: ADF4351 support
#define PLL_REFERENCE_FREQ 25 //MHz clock
// DB2OO: My ADF4351 supports min. Freq  of 32.4 MHz
#define PLL_FREQ_MIN 32000 //kHz
#define PLL_FREQ_MAX 4400000 //kHz
#else
#define PLL_REFERENCE_FREQ 10 //MHz clock
#define PLL_FREQ_MIN 137000 //kHz
#define PLL_FREQ_MAX 4400000 //kHz
#endif

#define PLL_POWER_MIN 0
#define PLL_POWER_MAX 3

#define CURSOR_POS_MIN 0
#define CURSOR_POS_MAX 7

unsigned long freq;
bool encode_sw_state;
unsigned long encode_debounce = 0;
bool cursor_blinking = false;
int8_t cursor_pos = 6;
int8_t pll_rf_power = 0;

void setup()
{
    int i;
  delay(1000);
  // ESP8266 has an issue with the Serial interface: need to start with another baud rate
#ifdef ESP8266
  Serial.begin(9600);
  while (! Serial) delay(1);
#endif
  Serial.begin(115200);
#ifdef ESP8266
  while (! Serial) delay(1);
#endif

  Wire.begin();
#ifdef SCAN_I2C
  Serial.println("\nI2C Scanner");
#endif  
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.home();
  lcd.print("Hello, world!");

  //spike for ADF4350 cold start
  for (i=0; i<1; i++) delay(5);

#ifndef SCAN_I2C

  freq = eeprom_read_dword(EEPROM_ADDR_FREQ);
  if (freq >= PLL_FREQ_MIN && freq <= PLL_FREQ_MAX) {
    Serial.print(F("Readed freq "));
    Serial.println(freq);

    cursor_pos = eeprom_read_byte(EEPROM_ADDR_CURPOS);
    if (cursor_pos > CURSOR_POS_MAX)cursor_pos = 0;

    pll_rf_power = eeprom_read_byte(EEPROM_ADDR_POWER);
    if (pll_rf_power > PLL_POWER_MAX)pll_rf_power = 0;

  } else freq = 432000;

  PLL.initialize(freq, PLL_REFERENCE_FREQ);
  PLL.setRfPower(pll_rf_power);

#ifdef HEAT_PIN
  pinMode(HEAT_PIN, INPUT);
#endif
  pinMode(LOCK_PIN, INPUT);
  pinMode(ENCODER_SW_PIN, INPUT);
  digitalWrite(ENCODER_SW_PIN, HIGH);

#endif  // SCAN__I2C

  printLCD();
  lcd.cursor();
  
#ifndef SCAN__I2C
  //DB2OO: set initial freq.
  PLL.setFreq(freq);
#endif  // SCAN__I2C
}

void printLCD()
{
  //long operation with screen blink
  //lcd.clear();

  //split 123456 to 123.456
  unsigned long part1 = freq / 1000;
  unsigned long part2 = freq - part1 * 1000;

  lcd.setCursor(0, 0);
  lcd.print("F =");
  if (part1 < 1000)lcd.print(" ");
  // DB2OO
  if (part1 < 100)lcd.print(" ");
  // DB2OO
  if (part1 < 10)lcd.print(" ");
  lcd.print(part1);
  lcd.print(".");
  if (part2 < 100)lcd.print("0");
  if (part2 < 10)lcd.print("0");
  lcd.print(part2);
  lcd.print(".000 ");

  lcd.setCursor(0, 1);
  lcd.print("P =");

  switch (pll_rf_power) {
    case 0:
      lcd.print("-4");
      break;
    case 1:
      lcd.print("-1");
      break;
    case 2:
      //DB2OO: 2 dBm, not 3
      lcd.print(" 2");
      break;
    case 3:
      lcd.print(" 5");
      break;
  }
  lcd.print("dBm");

  lcd.print(" ");

  if(rubid_heating){
    lcd.print("   HEAT");
  }else{
    if (!pll_lock){
      lcd.print("NO ");
    }else{
      lcd.print("   ");
    }
    lcd.print("LOCK");
  }

  if (cursor_pos <= 6) {
    lcd.setCursor(3 + cursor_pos + (cursor_pos > 3 ? 1 : 0), 0);
  } else {
    lcd.setCursor(0, 1);
  }
}

void loop()
{
#ifdef SCAN_I2C
  // I2C Scanner
  byte error, address;
  int nDevices;
  int i;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  for (i=0; i<1; i++) { yield(); delay(5);}
#else

  // DB2OO, 29.12.19: check Heat pin only, if defined
#ifdef HEAT_PIN
  if(rubid_heating != digitalRead(HEAT_PIN)){
    rubid_heating = !rubid_heating;
    PLL.update(); //for sure about that
    printLCD();
  }
#endif
  
  if (digitalRead(LOCK_PIN) != pll_lock) {
    pll_lock = !pll_lock;
    printLCD();
  }

  if (digitalRead(ENCODER_SW_PIN) != encode_sw_state) {
    delay(10);//debounce
    encode_sw_state = digitalRead(ENCODER_SW_PIN);

    //LOW = pressed
    //Serial.println(encode_sw_state?"HIGH":"LOW");

    if (encode_sw_state == LOW) {
      if (cursor_blinking) {
        lcd.noBlink();
        cursor_blinking = false;
      } else {
        lcd.blink();
        cursor_blinking = true;
      }
    }

  }

  long newPosition = encoder.read();
  if (newPosition != oldPosition) {
    float steps = (newPosition - oldPosition) / ENCODER_CLICK_STEP;
    Serial.println(steps);
    if (steps > 0.5 || steps < -0.5) {

      if (cursor_blinking) {
        //if in digit position mode
        if (steps > 0) {
          cursor_pos++;
        } else {
          cursor_pos--;
        }
        if (cursor_pos > CURSOR_POS_MAX)cursor_pos = CURSOR_POS_MAX;
        if (cursor_pos < CURSOR_POS_MIN)cursor_pos = CURSOR_POS_MIN;

      } else {
        if (cursor_pos <= 6) {
          //change frequency
          if (steps > 0) {
            //max step 1GHz
            freq += (1000000UL / pow(10, cursor_pos)) * ceil(steps);
          } else {
            freq += (1000000UL / pow(10, cursor_pos)) * floor(steps);
          }

          if (freq < PLL_FREQ_MIN)freq = PLL_FREQ_MIN;
          if (freq > PLL_FREQ_MAX)freq = PLL_FREQ_MAX;

          PLL.setFreq(freq);
        } else {
          //change power
          if (steps > 0) {
            pll_rf_power++;
          } else {
            pll_rf_power--;
          }
          if (pll_rf_power > PLL_POWER_MAX)pll_rf_power = PLL_POWER_MAX;
          if (pll_rf_power < PLL_POWER_MIN)pll_rf_power = PLL_POWER_MIN;

          PLL.setRfPower(pll_rf_power);
        }
        eeprom_write_timeout = millis();
      }
      printLCD();
      oldPosition = newPosition;
      encode_debounce = millis();
    }
    if (millis() - encode_debounce > 250) {
      oldPosition = newPosition;
      encode_debounce = millis();
    }
    //Serial.println(newPosition);
  }

  if ((eeprom_write_timeout > 0) && (millis() - eeprom_write_timeout > EEPROM_WRITE_TIMEOUT)) {
    eeprom_write_timeout = 0;
    Serial.print(F("Write EEPROM"));
    eeprom_write_dword(EEPROM_ADDR_FREQ, freq);
    eeprom_write_byte(EEPROM_ADDR_CURPOS, cursor_pos);
    eeprom_write_byte(EEPROM_ADDR_POWER, pll_rf_power);
  }
#endif  // SCAN_I2C
}
