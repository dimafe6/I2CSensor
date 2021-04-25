#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include "./src/GPIO.h"
#include "RF24.h"
#include "./src/BME280.h"
#include "./src/config.h"

const char *power_names[4] = {"MIN", "LOW", "HIGH", "MAX"};
const char *speed_names[3] = {"1MBPS", "2MBPS", "250KBPS"};
const uint64_t pipes[5] = {0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL};
const uint8_t bme_oversample_map[6] PROGMEM = {0, 1, 2, 4, 8, 16};
const uint8_t bme_filter_map[5] PROGMEM = {0, 2, 4, 8, 16};

RF24 radio(RADIO_CE, RADIO_CSN);
BME280 bme280;
SensorCalibration calibration;

#ifndef HARDWARE_VERSION_V3
signed int prevTemp;
signed int prevHum;
uint8_t prevBat;
uint8_t sleep_8s_count;
#endif

uint8_t from_node;
uint8_t rf_channel;
uint8_t rf_speed;
uint8_t rf_power;
uint8_t status_led_enabled;
uint8_t bme_filter;
uint8_t bme_temp_oversample;
uint8_t bme_hum_oversample;

#ifdef DISPLAY_MODE_PARTIAL_REDRAW
bool displayInit = false;
uint8_t displayUpdatesCount = 0;
#endif

struct ExternalSensor
{
  signed int temperature;
  signed int humidity;
  uint8_t battery;
};

ExternalSensor data;

void setup()
{
#ifdef HARDWARE_VERSION_V3
  pinAsOutput(TPL_DONE_PIN);
  digitalLow(TPL_DONE_PIN);
#else
  disableADC();
  disableSPI();
  disableTWI();
#endif

  pinAsInputPullUp(PROG_PIN);
  pinAsOutput(RF_PWR);
  pinAsOutput(SENSOR_PWR);
  pinAsOutput(CONFIG_LED);
  pinAsOutput(STATUS_LED);
  digitalLow(RF_PWR);
  digitalLow(SENSOR_PWR);
  digitalLow(CONFIG_LED);
  digitalLow(STATUS_LED);

  from_node = eeprom_read_byte((uint8_t *)EEPROM_NODE_ADDRESS);
  rf_channel = eeprom_read_byte((uint8_t *)EEPROM_CHANNEL_ADDRESS);
  rf_speed = eeprom_read_byte((uint8_t *)EEPROM_SPEED_ADDRESS);
  rf_power = eeprom_read_byte((uint8_t *)EEPROM_POWER_ADDRESS);
#ifndef HARDWARE_VERSION_V3
  sleep_8s_count = eeprom_read_byte((uint8_t *)EEPROM_SLEEP_TIME_ADDRESS);
#endif
  status_led_enabled = eeprom_read_byte((uint8_t *)EEPROM_ENABLE_STATUS_LED_ADDRESS);
  bme_filter = eeprom_read_byte((uint8_t *)EEPROM_BME_FILTER_ADDRESS);
  bme_temp_oversample = eeprom_read_byte((uint8_t *)EEPROM_BME_TEMP_OVERSAMPLE_ADDRESS);
  bme_hum_oversample = eeprom_read_byte((uint8_t *)EEPROM_BME_HUM_OVERSAMPLE_ADDRESS);

  bool configIsValid = (from_node >= 1 && from_node <= 5) &&
                       (rf_channel >= 1 && rf_channel <= 125) &&
                       (rf_speed >= 0 && rf_speed <= 2) &&
                       (rf_power >= 0 && rf_speed <= 3) &&
#ifndef HARDWARE_VERSION_V3
                       (sleep_8s_count >= 1 && sleep_8s_count <= 432000) &&
#endif
                       (bme_filter >= 0 && bme_filter <= 4) &&
                       (bme_temp_oversample >= 0 && bme_temp_oversample <= 5) &&
                       (bme_hum_oversample >= 0 && bme_hum_oversample <= 5);

#ifdef DEBUG
  Serial.begin(9600);
#endif

  if (!configIsValid)
  {
    from_node = 1;
    rf_channel = 80;
    rf_speed = 2;
    rf_power = 3;
#ifndef HARDWARE_VERSION_V3
    sleep_8s_count = 1;
#endif
    status_led_enabled = 1;
    bme_filter = 2;
    bme_temp_oversample = 2;
    bme_hum_oversample = 2;
  }

  if (isLow(PROG_PIN))
  {
    Serial.begin(9600);

    Serial.println(F("\nConfiguration mode"));

    digitalHigh(CONFIG_LED);

#ifdef ENABLE_DISPLAY
    enableSPI();
    digitalHigh(RF_PWR);
    SPI.begin();
    digitalLow(RADIO_CE);
    digitalHigh(RADIO_CSN);
    digitalLow(RF_PWR);

#ifdef DEBUG
    display.init(9600);
#else
    display.init();
#endif
    display.drawPaged(showConfigCallback, 0);
#endif

    // TODO: Add display rotation to config
    configure();
    digitalLow(CONFIG_LED);

#ifdef ENABLE_DISPLAY
    display.drawPaged(showConfigCallback, 0);
    disableSPI();
#endif
  }
  else
  {
#ifdef DEBUG
    printConfig();
#endif
  }
}

void loop()
{
#ifdef DEBUG
  Serial.println(F("Start"));
  unsigned long startTime = millis();
#endif

#ifndef HARDWARE_VERSION_V3
  enableADC();
  prevBat = data.battery;
#endif
  data.battery = getBatteryPercent();
#ifndef HARDWARE_VERSION_V3
  disableADC();
#endif

  enableStatusLED();

  digitalHigh(SENSOR_PWR);
  enableTWI();

  delay(2);

  if (bme280.begin(0x76, &calibration) == false)
  {
#ifdef DEBUG
    Serial.println(F("Sensor connect failed"));
#endif

    data.temperature = 0;
    data.humidity = 0;
    data.battery = 0;

    for (uint8_t i = 0; i <= 3; i++)
    {
      enableStatusLED();
      delay(200);
      disableStatusLED();
      delay(200);
    }

    digitalLow(SENSOR_PWR);
    disableStatusLED();

    powerDown();
  }
  else
  {
    bme280.setFilter(pgm_read_byte(&bme_filter_map[bme_filter]));
    bme280.setTempOverSample(pgm_read_byte(&bme_oversample_map[bme_temp_oversample]));
    bme280.setHumidityOverSample(pgm_read_byte(&bme_oversample_map[bme_hum_oversample]));
    bme280.setMode(MODE_FORCED);

    while (bme280.isMeasuring())
    {
      delay(1);
    };

#ifndef HARDWARE_VERSION_V3
    prevTemp = (int)(data.temperature / 100);
    prevHum = (int)(data.humidity / 100);
#endif

    data.temperature = bme280.readTempC() * 100;
    data.humidity = bme280.readFloatHumidity() * 100;

    disableTWI();
    digitalLow(SENSOR_PWR);

    enableSPI();
    digitalHigh(RF_PWR);

    radio.begin();
    radio.powerUp();
    radio.stopListening();
    radio.setAutoAck(false);
    radio.setChannel(rf_channel);
    radio.setPayloadSize(sizeof(data));
    radio.setPALevel(rf24_pa_dbm_e(rf_power));
    radio.setDataRate(rf24_datarate_e(rf_speed));
    radio.disableCRC();
    radio.openWritingPipe(pipes[from_node - 1]);

    radio.write(&data, sizeof(data));

    radio.powerDown();
    digitalLow(RF_PWR);
    disableStatusLED();

#ifdef DEBUG
    Serial.print(F("Hum: "));
    Serial.print(data.humidity);
    Serial.println();

    Serial.print(F("Temp: "));
    Serial.print(data.temperature);
    Serial.println();

    Serial.print(F("Bat: "));
    Serial.print(data.battery);
    Serial.print('%');
    Serial.println();
#endif

#ifdef ENABLE_DISPLAY
    bool tempChanged = prevTemp != (int)(data.temperature / 100);
    bool humChanged = prevHum != (int)(data.humidity / 100);
    bool batChanged = map(prevBat, 0, 100, 3, 28) != map(data.battery, 0, 100, 3, 28);

#if defined(DISPLAY_MODE_FULL_REDRAW)
    if (tempChanged || humChanged || batChanged)
    {
#ifdef DEBUG
      display.init(9600);
#else
      display.init();
#endif
      display.drawPaged(showGUICallback, 0);
    }
#elif defined(DISPLAY_MODE_PARTIAL_REDRAW)
    if (!displayInit || (FULL_REDRAW_AFTER_UPDATES > 0 ? displayUpdatesCount >= FULL_REDRAW_AFTER_UPDATES : false))
    {
#ifdef DEBUG
      display.init(9600);
#else
      display.init();
#endif
      display.mirror(false);
      display.setRotation(DISPLAY_ROTATION);
      display.drawPaged(showGUICallback, 0);
      display.setFont(&DSEG7_Classic_Bold_64);
      displayInit = true;
      displayUpdatesCount = 0;
    }
    else
    {
      if (tempChanged)
      {
        display.setPartialWindow(52, 20, 98, 70);
        display.drawPaged(showTempBoxCallback, 0);
      }

      if (humChanged)
      {
        display.setPartialWindow(52, 120, 98, 70);
        display.drawPaged(showHumBoxCallback, 0);
      }

      if (batChanged)
      {
        display.setPartialWindow(5, 175, 40, 24);
        display.drawPaged(showBatBoxCallback, 0);
      }

      if (tempChanged || humChanged || batChanged)
      {
        displayUpdatesCount++;
      }
    }
#endif
    display.hibernate();
    display.powerOff();
#endif
  }

#ifdef DEBUG
  Serial.print(F("Running time(ms): "));
  Serial.println(millis() - startTime);
  Serial.println(F("Sleep"));
  delay(1000); // Delay for complete Serial write
#endif

  powerDown();
}

#ifdef ENABLE_DISPLAY

void showGUICallback(const void *parameters)
{
  display.setTextColor(GxEPD_BLACK);
  display.mirror(false);

  display.drawRect(0, 0, 200, 200, GxEPD_BLACK);
  display.drawLine(0, 8, 22, 8, GxEPD_BLACK);
  display.drawLine(178, 8, 200, 8, GxEPD_BLACK);

  display.setFont(&DSEG7_Classic_Bold_64);
  display.setCursor(48, 87);
  display.print((int)(data.temperature / 100));

  display.setCursor(48, 187);
  display.print((int)(data.humidity / 100));

  display.drawCircle(160, 35, 2, GxEPD_BLACK);
  display.drawCircle(160, 35, 3, GxEPD_BLACK);
  display.drawCircle(160, 35, 4, GxEPD_BLACK);

  display.setFont(&FreeMonoBold9pt7b);
  display.setCursor(39, 15);

  display.print(F("TEMPERATURE"));

  display.setCursor(170, 50);
  display.print('C');

  display.drawLine(0, 108, 32, 108, GxEPD_BLACK);
  display.drawLine(168, 108, 200, 108, GxEPD_BLACK);

  display.setCursor(60, 115);
  display.print(F("HUMIDITY"));

  display.setFont(&percentFont);
  display.setCursor(160, 145);
  display.print('%');

  display.drawLine(0, 175, 40, 175, GxEPD_BLACK);
  display.drawRect(5, 182, 30, 12, GxEPD_BLACK);
  display.fillRect(35, 185, 2, 6, GxEPD_BLACK);
  display.fillRect(37, 187, 1, 2, GxEPD_BLACK);
  display.fillRect(6, 184, (uint8_t)map(data.battery, 0, 100, 3, 28), 8, GxEPD_BLACK);
  display.drawLine(40, 175, 50, 200, GxEPD_BLACK);

  display.drawLine(160, 175, 200, 175, GxEPD_BLACK);
  display.setFont(&FreeMonoBold9pt7b);
  display.setCursor(157, 193);
  display.print(F("ID:"));
  display.setCursor(186, 193);
  display.print(from_node);
  display.drawLine(160, 175, 150, 200, GxEPD_BLACK);
}

#ifdef DISPLAY_MODE_PARTIAL_REDRAW
void showTempBoxCallback(const void *parameters)
{
  display.setCursor(48, 87);
  display.print((int)(data.temperature / 100));
}

void showHumBoxCallback(const void *parameters)
{
  display.setCursor(48, 187);
  display.print((int)(data.humidity / 100));
}

void showBatBoxCallback(const void *parameters)
{
  display.drawLine(0, 175, 0, 200, GxEPD_BLACK);
  display.drawLine(0, 175, 40, 175, GxEPD_BLACK);
  display.drawRect(5, 182, 30, 12, GxEPD_BLACK);
  display.fillRect(35, 185, 2, 6, GxEPD_BLACK);
  display.fillRect(37, 187, 1, 2, GxEPD_BLACK);
  display.fillRect(6, 184, (uint8_t)map(data.battery, 0, 100, 3, 28), 8, GxEPD_BLACK);
  display.drawLine(40, 175, 50, 200, GxEPD_BLACK);
}
#endif

void showConfigCallback(const void *parameters)
{
  display.setTextColor(GxEPD_BLACK);
  display.mirror(false);
  display.setFont(&FreeMonoBold9pt7b);
  display.setCursor(0, 0);

  display.println();

  display.print(F("ID"));
  display.println(from_node);

  display.print(F("Channel:"));
  display.println(rf_channel);

  display.print(F("Speed:"));
  display.println(speed_names[rf_speed]);

  display.print(F("Power:"));
  display.println(power_names[rf_power]);

#ifndef HARDWARE_VERSION_V3
  display.print(F("Sleep:"));
  display.print(sleep_8s_count * 8);
  display.print(F("sec"));
#endif
}

#endif

void configure()
{
  while (!Serial)
  {
    ; // wait for serial port to connect.
  }

  Serial.flush();

  while (1)
  {
    Serial.println(F("*** Node address (1-5):"));
    while (!Serial.available())
      ;

    from_node = Serial.readStringUntil('\n').toInt();

    if (from_node >= 1 && from_node <= 5)
    {
      eeprom_write_byte((uint8_t *)EEPROM_NODE_ADDRESS, from_node);
      Serial.println(from_node);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Node channel id (1-125):"));
    while (!Serial.available())
      ;

    rf_channel = Serial.readStringUntil('\n').toInt();

    if (rf_channel >= 1 && rf_channel <= 125)
    {
      eeprom_write_byte((uint8_t *)EEPROM_CHANNEL_ADDRESS, rf_channel);
      Serial.println(rf_channel);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Radio speed (0-2):\n0 - 1MBPS\r\n1 - 2MBPS\r\n2 - 250KBPS"));
    while (!Serial.available())
      ;

    rf_speed = Serial.readStringUntil('\n').toInt();

    if (rf_speed >= 0 && rf_speed <= 2)
    {
      eeprom_write_byte((uint8_t *)EEPROM_SPEED_ADDRESS, rf_speed);
      Serial.println(rf_speed);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Radio power (0-3):"));
    Serial.println(F("0 - MIN\r\n1 - LOW\r\n2 - HIGH\r\n3 - MAX"));
    while (!Serial.available())
      ;

    rf_power = Serial.readStringUntil('\n').toInt();

    if (rf_power >= 0 && rf_speed <= 3)
    {
      eeprom_write_byte((uint8_t *)EEPROM_POWER_ADDRESS, rf_power);
      Serial.println(rf_power);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** BME280 filter level:"));
    Serial.println(F("0 - Disabled\r\n1 - 2x\r\n2 - 4x\r\n3 - 8x\r\n4 - 16x"));
    while (!Serial.available())
      ;

    bme_filter = Serial.readStringUntil('\n').toInt();

    if (bme_filter >= 0 && bme_filter <= 4)
    {
      eeprom_write_byte((uint8_t *)EEPROM_BME_FILTER_ADDRESS, bme_filter);
      Serial.println(bme_filter);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** BME280 temperature oversample:"));
    Serial.println(F("0 - Disabled\r\n1 - 1x\r\n2 - 2x\r\n3 - 4x\r\n4 - 8x\r\n5 - 16x"));
    while (!Serial.available())
      ;

    bme_temp_oversample = Serial.readStringUntil('\n').toInt();

    if (bme_temp_oversample >= 0 && bme_temp_oversample <= 5)
    {
      eeprom_write_byte((uint8_t *)EEPROM_BME_TEMP_OVERSAMPLE_ADDRESS, bme_temp_oversample);
      Serial.println(bme_temp_oversample);
      break;
    }
    else
    {
      Serial.println(F(""));
    }
  }

  while (1)
  {
    Serial.println(F("*** Select BME280 humidity oversample:"));
    Serial.println(F("0 - Disabled\r\n1 - 1x\r\n2 - 2x\r\n3 - 4x\r\n4 - 8x\r\n5 - 16x"));
    while (!Serial.available())
      ;

    bme_hum_oversample = Serial.readStringUntil('\n').toInt();

    if (bme_hum_oversample >= 0 && bme_hum_oversample <= 5)
    {
      eeprom_write_byte((uint8_t *)EEPROM_BME_HUM_OVERSAMPLE_ADDRESS, bme_hum_oversample);
      Serial.println(bme_hum_oversample);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Enter sleep time in seconds that is divisible by 8:"));
    Serial.println(F("Ex. 8, 16, 32, 128"));
    while (!Serial.available())
      ;

    int sleep_time = Serial.readStringUntil('\n').toInt();
#ifndef HARDWARE_VERSION_V3
    if (sleep_time >= 8 && sleep_time <= 432000 && (sleep_time % 8) == 0)
    {
      sleep_8s_count = sleep_time / 8;
      eeprom_write_byte((uint8_t *)EEPROM_SLEEP_TIME_ADDRESS, sleep_8s_count);
      Serial.println(sleep_time);
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
#endif
  }

  while (1)
  {
    Serial.println(F("*** Enable status LED?"));
    Serial.println(F("1 - Enable\r\n0 - Disable"));
    while (!Serial.available())
      ;

    status_led_enabled = Serial.readStringUntil('\n').toInt();

    if (status_led_enabled >= 0 && status_led_enabled < 2)
    {
      eeprom_write_byte((uint8_t *)EEPROM_ENABLE_STATUS_LED_ADDRESS, status_led_enabled);
      Serial.println((status_led_enabled == 1) ? F("Enabled") : F("Disabled"));
      break;
    }
    else
    {
      Serial.println(F("Wrong value!"));
    }
  }

  printConfig();
}

void printConfig()
{
  Serial.println();
  Serial.print(F("ID"));
  Serial.print(from_node);
  Serial.println();
  Serial.print(F("Channel:"));
  Serial.print(rf_channel);
  Serial.println();
  Serial.print(F("Speed:"));
  Serial.print(speed_names[rf_speed]);
  Serial.println();
  Serial.print(F("Power:"));
  Serial.print(power_names[rf_power]);
  Serial.println();
  Serial.print(F("BME280 filter level: "));
  if (pgm_read_byte(&bme_filter_map[bme_filter]) > 0)
  {
    Serial.print(pgm_read_byte(&bme_filter_map[bme_filter]));
    Serial.print('x');
  }
  else
  {
    Serial.print(F("Disabled"));
  }
  Serial.println();
  Serial.print(F("BME280 temperature oversample: "));
  if (pgm_read_byte(&bme_oversample_map[bme_temp_oversample]) > 0)
  {
    Serial.print(pgm_read_byte(&bme_oversample_map[bme_temp_oversample]));
    Serial.print('x');
  }
  else
  {
    Serial.print(F("Disabled"));
  }
  Serial.println();
  Serial.print(F("BME280 humidity oversample: "));
  if (pgm_read_byte(&bme_oversample_map[bme_hum_oversample]) > 0)
  {
    Serial.print(pgm_read_byte(&bme_oversample_map[bme_hum_oversample]));
    Serial.print('x');
  }
  else
  {
    Serial.print(F("Disabled"));
  }
#ifndef HARDWARE_VERSION_V3
  Serial.println();
  Serial.print(F("Sleep:"));
  Serial.print(sleep_8s_count * 8);
  Serial.println();
  Serial.print(F("Status LED: "));
  Serial.print((status_led_enabled == 1) ? F("Enabled") : F("Disabled"));
  Serial.println();
#endif
  delay(1000);
}

void enableStatusLED()
{
  if (status_led_enabled)
  {
    digitalHigh(STATUS_LED);
  }
}

void disableStatusLED()
{
  if (status_led_enabled)
  {
    digitalLow(STATUS_LED);
  }
}

uint16_t readVcc()
{
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;

  uint8_t low = ADCL;
  uint8_t high = ADCH;

  uint16_t result = (high << 8) | low;

  return 1108352L / result;
}

uint8_t getBatteryPercent()
{
  float batteryV = readVcc();
  int batteryPcnt = (((batteryV - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100);

  return constrain(batteryPcnt, 0, 100);
}

void disableADC()
{
  ADCSRA = 0;
  power_adc_disable();
}

void enableADC()
{
  power_adc_enable();
  ADCSRA |= 1 << ADEN;
}

void enableSPI()
{
  power_spi_enable();
  SPCR |= 1 << SPE;
}

void disableSPI()
{
  SPCR &= ~_BV(SPE);
  power_spi_disable();
}

void enableTWI()
{
  power_twi_enable();
}

void disableTWI()
{
  power_twi_disable();
  TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));
  digitalLow(SDA);
  digitalLow(SCL);
}

#ifndef HARDWARE_VERSION_V3
ISR(WDT_vect)
{
  wdt_disable();
}
#endif

void powerDown()
{
#ifdef HARDWARE_VERSION_V3
  while (1)
  {
    digitalHigh(TPL_DONE_PIN);
    delayMicroseconds(1);
    digitalLow(TPL_DONE_PIN);
    delayMicroseconds(1);
  }
#else
  unsigned int sleepCounter;
  for (sleepCounter = sleep_8s_count; sleepCounter > 0; sleepCounter--)
  {
    disableADC();

    wdt_enable(WDTO_8S);
    WDTCSR |= (1 << WDIE);

    do
    {
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      cli();
      sleep_enable();
      sleep_bod_disable();
      sei();
      sleep_cpu();
      sleep_disable();
      sei();
    } while (0);
    disableADC();
  }
#endif
}