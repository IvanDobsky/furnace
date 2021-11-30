#include <EEPROM.h>
/*
 * Необходимо записать данные(любые)в следующие ячейки:
 * 0, 4, 8, 20, 24, 28, 32, 36
 * */
void setup() {
   /*
  for(int i=0; i<40; i++){
    EEPROM.put(i, 255);
  }
 */
   
  EEPROM.put(0, 1.1);  
  EEPROM.put(4, 0.1);
  EEPROM.put(8, 1.2);
  EEPROM.put(20, 0);
  EEPROM.put(24, 120);
  EEPROM.put(28, 2);
  EEPROM.put(32, 1);
  EEPROM.put(36, 1);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
