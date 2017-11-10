#include <EEPROM.h>
//#include <p18f452.h>

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;
int top =0, bottom = 0;
int addr=0;
double out = 0;

void setup() {
  Serial.begin(38400);
}

void loop() {
  for (int j = 0; j < 40; j++){
        //EECON1bits.CFGS=0;
        EEPROM.get(addr, out);
        Serial.println(out);
        addr += sizeof(double);
        
      }
      delay(600000);

}
