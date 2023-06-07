//////////////////////////////////////////////////////////
///
//  Copyright (c) 2023
//  Author: Jacob Garner, mlgtex4127@gmail.com
//  
//  Filename: TeensyBME280.cpp
//
//  Description:
//  This code is designed to integrate functionality of the BME280 chip manufactured
//  by BOSCH for the Teensy. SPI and I2C functionality is now supported for use with 
//  the Teensy 4.1 micro controller.
///
//////////////////////////////////////////////////////////

#include "TeensyBME280.h"
#include "Arduino.h"
#include <SPI.h>

////////////////////////////CONSTRUCTOR////////////////////////////////////////

TeensyBME280::TeensyBME280(int8_t cspin, int8_t mosipin, int8_t misopin,
    int8_t sckpin) :

    _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin) {}
//note that the SPI call is explicit for this setup, no implicit workaround worked consitently}:

////////////////////////////DESTRUCTOR////////////////////////////////////////

TeensyBME280::~TeensyBME280(void)
{
  ///destroys all instances of the sensors}
  if (temp_sensor) {
    delete temp_sensor;
  }
  if (pressure_sensor) {
    delete pressure_sensor;
  }
  if (humidity_sensor) {
    delete humidity_sensor;
  }
 
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

bool TeensyBME280::init()
{
  //AVOID USING WIRE, DUE TO EXPLICIT INSTANTIATION, NO NEED TO WORRY  ABOUT OTHER WAYS TO CALL OBJECT
  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);
  pinMode(_sck, OUTPUT);
  pinMode(_mosi, OUTPUT);
  pinMode(_miso, INPUT);

  _sensorID = read8(BME280_REGISTER_CHIPID);
  if (_sensorID != 0x60)
  {
  return false;
  }
  write8(BME280_REGISTER_SOFTRESET, 0xB6);
  delay(10);
  while(isReadingCalibration())
  {
    delay(10);
  }
  readCoefficients();
  setSampling();
  delay(100);
  return true;
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

void TeensyBME280::setSampling(sensor_mode mode,
                 sensor_sampling tempSampling,
                 sensor_sampling pressSampling,
                 sensor_sampling humSampling,
                 sensor_filter filter,
                 standby_duration duration)
{
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _humReg.osrs_h = humSampling;
  _configReg.filter = filter;
  _configReg.t_sb = duration;
  //put the chip to sleep so it will take configuration
  write8(BME280_REGISTER_CONTROL, MODE_SLEEP);
  //set register contorl as outlined in datasheed 5.4.3
  write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
  write8(BME280_REGISTER_CONFIG, _configReg.get());
  write8(BME280_REGISTER_CONTROL, _measReg.get());
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

//x parameter is the byte to be transfered

uint8_t TeensyBME280::spixfer(uint8_t x)
{
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) ////we are reading the bits in reverse
  {
    reply <<=1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1 << i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

void TeensyBME280::write8(byte reg, byte value)
{
  digitalWrite(_cs, LOW);
  spixfer(reg & ~0x80); // write, bit 7 low
  spixfer(value);
  digitalWrite(_cs, HIGH);
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

uint8_t TeensyBME280::read8(byte reg)
{
  uint8_t value;
  digitalWrite(_cs, LOW);
  spixfer(reg | 0x80); // read, bit 7 high
  value = spixfer(0);
  digitalWrite(_cs, HIGH);
  
  return value;
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

uint16_t TeensyBME280::read16(byte reg)
{
  uint16_t value;
  digitalWrite(_cs, LOW);
  spixfer(reg | 0x80); // read, bit 7 high
  value = (spixfer(0) << 8) | spixfer(0);
  digitalWrite(_cs, HIGH);

  return value;
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

uint16_t TeensyBME280::read16_LE(byte reg)
{
  uint8_t temp = read16(reg);
  return (temp<<8) | (temp << 8);
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

int16_t TeensyBME280::readS16(byte reg)
{
  return (int16_t)read16(reg);
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

int16_t TeensyBME280::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

uint32_t TeensyBME280::read24(byte reg)
{
  uint32_t value;
  digitalWrite(_cs, LOW);
  spixfer(reg | 0x80); // read, bit 7 high

  value = spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  value <<= 8;
  value |= spixfer(0);

  digitalWrite(_cs, HIGH);

  return value;
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

bool TeensyBME280::takeForcedMeasurement(void)
{
  bool return_value = false;
  if (_measReg.mode == MODE_FORCED) {
    return_value = true;
    // set to forced mode, i.e. "take next measurement"
    write8(BME280_REGISTER_CONTROL, _measReg.get());
    // Store current time to measure the timeout
    uint32_t timeout_start = millis();
    // wait until measurement has been completed, otherwise we would read the
    // the values from the last measurement or the timeout occurred after 2 sec.
    while (read8(BME280_REGISTER_STATUS) & 0x08) {
      // In case of a timeout, stop the while loop
      if ((millis() - timeout_start) > 2000) {
        return_value = false;
        break;
      }
      delay(1);
    }
  }
  return return_value;
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

void TeensyBME280::readCoefficients(void)
{
  _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
  _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
  _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

  _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
  _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
  _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
  _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
  _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
  _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
  _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
  _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
  _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

  _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
  _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
  _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
  _bme280_calib.dig_H4 = ((int8_t)read8(BME280_REGISTER_DIG_H4) << 4) |
                         (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
  _bme280_calib.dig_H5 = ((int8_t)read8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                         (read8(BME280_REGISTER_DIG_H5) >> 4);
  _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

////////////////////////////NEW FUNCTION////////////////////////////////////////

bool TeensyBME280::isReadingCalibration(void)
{
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

   return (rStatus & (1 << 0)) != 0;
}

////////////////////////////NEW FUNCTION -- READING DATA////////////////////////////////////////

float TeensyBME280::readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
  if (adc_T == 0x800000) // value in case temp measurement was disabled
    return NAN;
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
          ((int32_t)_bme280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bme280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2 + t_fine_adjust;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

////////////////////////////NEW FUNCTION -- READING DATA////////////////////////////////////////

float TeensyBME280::readPressure(void)
{
  int64_t var1, var2, p;

  readTemperature(); // must be done first to get t_fine

  int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
  if (adc_P == 0x800000) // value in case pressure measurement was disabled
    return NAN;
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
  return (float)p / 256;
}

////////////////////////////NEW FUNCTION -- READING DATA////////////////////////////////////////

float TeensyBME280::readHumidity(void)
{
  readTemperature(); // must be done first to get t_fine

  int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
  if (adc_H == 0x8000) // value in case humidity measurement was disabled
    return NAN;

  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) +
                     ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)_bme280_calib.dig_H2) +
                 8192) >>
                14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)_bme280_calib.dig_H1)) >>
                            4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r >> 12);
  return h / 1024.0;
}

////////////////////NOTE---OMISION OF ALTITUDE///////////////////////
//DUE TO NOT NEEDING TO KNOW ALTITUDE, THIS HAS BEEN REMOVED FROM THE FUNCTION OF THIS LIBRARY; SHOULD IT BE NEEDED, PLACE HERE!!!!//

////////////////////////////NEW FUNCTION -- GETTERS AND SETTERS!!!////////////////////////////////////////

uint32_t TeensyBME280::sensorID(void) {return _sensorID;}
float TeensyBME280::getTemperatureCompensation(void) {return float(((t_fine_adjust *5) >> 8)/100);}
void TeensyBME280::setTemperatureCompensation(float value) {t_fine_adjust = ((int32_t(value * 100) << 8)) / 5;}
Adafruit_Sensor* TeensyBME280::getTemperatureSensor(void){ if(!temp_sensor){temp_sensor = new TeensyBME280_Temp(this);} return temp_sensor;}
Adafruit_Sensor* TeensyBME280::getPressureSensor(void){ if(!pressure_sensor){pressure_sensor = new TeensyBME280_Pressure(this);} return pressure_sensor;}
Adafruit_Sensor * TeensyBME280::getHumiditySensor(void){ if(!humidity_sensor){humidity_sensor = new TeensyBME280_Humidity(this);} return humidity_sensor;}

void TeensyBME280_Temp::getSensor(sensor_t *sensor)
{
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "BME280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40.0; /* Temperature range -40 ~ +85 C  */
  sensor->max_value = +85.0;
  sensor->resolution = 0.01; /*  0.01 C */
}

bool TeensyBME280_Temp::getEvent(sensors_event_t *event)
{
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _theBME280->readTemperature();
  return true;
}

void TeensyBME280_Pressure::getSensor(sensor_t *sensor)
{
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "BME280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->min_value = 300.0; /* 300 ~ 1100 hPa  */
  sensor->max_value = 1100.0;
  sensor->resolution = 0.012; /* 0.12 hPa relative */
}

bool TeensyBME280_Pressure::getEvent(sensors_event_t *event)
{
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = millis();
  event->pressure = _theBME280->readPressure() / 100; // convert Pa to hPa
  return true; 
}

void TeensyBME280_Humidity::getSensor(sensor_t *sensor)
{
  memset(sensor , 0 , sizeof(sensor_t));
  strncpy(sensor->name, "BME280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->min_delay = 0;
  sensor->min_value = 0;
  sensor->max_value = 100; /* 0 - 100 %  */
  sensor->resolution = 3;  /* 3% accuracy */
}

bool TeensyBME280_Humidity::getEvent(sensors_event_t *event)
{
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  event->timestamp = millis();
  event->relative_humidity = _theBME280->readHumidity();
  return true;
}
