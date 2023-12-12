#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <MCP3421.h>

//Voids
//Objects Init
LiquidCrystal_I2C lcd(0X27,20,4);
CMCP3421 adc(0.1692);//Esto es lo que no entiendo
//Variables
float   fValue;
int32_t s32Value;


void setup() {
  Wire.begin();
  Serial.begin(115200);
  adc.Init();
  adc.Trigger();
  lcd.init();
  lcd.backlight();
  lcd.print("Version 01");  
}

void loop() {
  if (adc.IsReady())
  {
    fValue   = adc.ReadValue();
    s32Value = adc.ReadRaw();
    lcd.setCursor(0,1);
    lcd.printf("ADC: %1d     ",s32Value);
    lcd.setCursor(0,2);
    lcd.printf("MiliVolts: %.2f      ", fValue/10);
    adc.Trigger();
  }
}

