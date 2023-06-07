///simple test of code to read out and translate the readings for the BME280

#include <RS_BME280.h>

#define CSPIN
#define MOSIPIN
#define MISOPIN
#define SCKPIN

RS_BME280 bme(CSPIN, MOSIPIN, MISOPIN, SCKPIN);

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  bme.init();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  
}
