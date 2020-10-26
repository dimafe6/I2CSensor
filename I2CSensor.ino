#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include "./src/RF24.h"
#include "./src/BME280.h"

//#define DEBUG

#define EEPROM_NODE_ADDRESS 0
#define EEPROM_CHANNEL_ADDRESS 1
#define EEPROM_SPEED_ADDRESS 2
#define EEPROM_POWER_ADDRESS 3
#define EEPROM_SLEEP_TIME_ADDRESS 4
#define EEPROM_ENABLE_STATUS_LED_ADDRESS 5
#define EEPROM_BME_FILTER_ADDRESS 6
#define EEPROM_BME_TEMP_OVERSAMPLE_ADDRESS 7
#define EEPROM_BME_HUM_OVERSAMPLE_ADDRESS 8

#define PROG_PIN 2
#define RADIO_CE 9
#define RADIO_CSN 10
#define RF_PWR PD5
#define SENSOR_PWR 4
#define CONFIG_LED A1
#define STATUS_LED A2

const char *power_names[4] = {"MIN", "LOW", "HIGH", "MAX"};
const char *speed_names[3] = {"1MBPS", "2MBPS", "250KBPS"};
const uint64_t pipes[5] = {0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL};
const uint8_t bme_oversample_map[6] PROGMEM = {0, 1, 2, 4, 8, 16};
const uint8_t bme_filter_map[5] PROGMEM = {0, 2, 4, 8, 16};

RF24 radio(RADIO_CE, RADIO_CSN);
BME280 bme280;
SensorCalibration calibration;

uint8_t from_node;
uint8_t rf_channel;
uint8_t rf_speed;
uint8_t rf_power;
uint8_t sleep_8s_count;
uint8_t status_led_enabled;
uint8_t bme_filter;
uint8_t bme_temp_oversample;
uint8_t bme_hum_oversample;
uint16_t battery_min_voltage = 2000;
uint16_t battery_max_voltage = 3600;

struct ExternalSensor
{
  signed int temperature;
  signed int humidity;
  uint8_t battery;
};

ExternalSensor data;

void setup()
{
  disableADC();
  disableSPI();
  disableTWI();

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
  sleep_8s_count = eeprom_read_byte((uint8_t *)EEPROM_SLEEP_TIME_ADDRESS);
  status_led_enabled = eeprom_read_byte((uint8_t *)EEPROM_ENABLE_STATUS_LED_ADDRESS);
  bme_filter = eeprom_read_byte((uint8_t *)EEPROM_BME_FILTER_ADDRESS);
  bme_temp_oversample = eeprom_read_byte((uint8_t *)EEPROM_BME_TEMP_OVERSAMPLE_ADDRESS);
  bme_hum_oversample = eeprom_read_byte((uint8_t *)EEPROM_BME_HUM_OVERSAMPLE_ADDRESS);

  bool configIsValid = (from_node >= 1 && from_node <= 5) &&
                       (rf_channel >= 1 && rf_channel <= 125) &&
                       (rf_speed >= 0 && rf_speed <= 2) &&
                       (rf_power >= 0 && rf_speed <= 3) &&
                       (sleep_8s_count >= 1 && sleep_8s_count <= 432000) &&
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
    sleep_8s_count = 1;
    status_led_enabled = 1;
    bme_filter = 2;
    bme_temp_oversample = 2;
    bme_hum_oversample = 2;
  }

  if (isLow(PROG_PIN))
  {
    Serial.begin(9600);
    Serial.println("");

    Serial.println(F("Configuration mode"));

    digitalHigh(CONFIG_LED);
    configure();
    digitalLow(CONFIG_LED);
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

  enableADC();
  data.battery = getBatteryPercent();
  disableADC();

  enableStatusLED();

  digitalHigh(SENSOR_PWR);
  enableTWI();

  delay(2);

  if (bme280.begin(0x76, &calibration) == false)
  {
#ifdef DEBUG
    Serial.println(F("BME280 sensor connect failed"));
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

    data.temperature = bme280.readTempC() * 100;
    data.humidity = bme280.readFloatHumidity() * 100;

    disableTWI();
    digitalLow(SENSOR_PWR);

    digitalHigh(RF_PWR);
    enableSPI();

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
    Serial.print(F("Humidity: "));
    Serial.print(data.humidity);
    Serial.println();

    Serial.print(F("Temp: "));
    Serial.print(data.temperature);
    Serial.println();

    Serial.print(F("Bat: "));
    Serial.print(data.battery);
    Serial.print('%');
    Serial.println();

    Serial.print(F("Running time(ms): "));
    Serial.println(millis() - startTime);
    Serial.println(F("Going to sleep"));
    delay(1000); // Delay for complete Serial write
#endif

    unsigned int sleepCounter;
    for (sleepCounter = sleep_8s_count; sleepCounter > 0; sleepCounter--)
    {
      powerDown();
    }
  }
}

void configure()
{
  while (!Serial)
  {
    ; // wait for serial port to connect.
  }

  while (1)
  {
    Serial.println(F("*** Enter node address (1-5):"));
    while (!Serial.available())
      ;

    from_node = Serial.parseInt();

    if (from_node >= 1 && from_node <= 5)
    {
      eeprom_write_byte((uint8_t *)EEPROM_NODE_ADDRESS, from_node);
      Serial.println(from_node);
      break;
    }
    else
    {
      Serial.println(F("Wrong node address"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Enter node channel id (1-125):"));
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
      Serial.println(F("Wrong channel id"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Select radio speed (0-2):"));
    Serial.println(F("0 - 1MBPS\r\n1 - 2MBPS\r\n2 - 250KBPS"));
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
      Serial.println(F("Wrong radio speed"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Select radio power (0-3):"));
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
      Serial.println(F("Wrong radio power"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Select BME280 filter level:"));
    Serial.println(F("0 - Filter off\r\n1 - Coefficients 2\r\n2 - Coefficients 4\r\n3 - Coefficients 8\r\n4 - Coefficients 16"));
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
      Serial.println(F("Wrong BME280 filter level"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Select BME280 temperature oversample:"));
    Serial.println(F("0 - Oversample disabled\r\n1 - Coefficients 1\r\n2 - Coefficients 2\r\n3 - Coefficients 4\r\n4 - Coefficients 8\r\n5 - Coefficients 16"));
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
      Serial.println(F("Wrong BME280 temperature oversample"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Select BME280 humidity oversample:"));
    Serial.println(F("0 - Oversample disabled\r\n1 - Coefficients 1\r\n2 - Coefficients 2\r\n3 - Coefficients 4\r\n4 - Coefficients 8\r\n5 - Coefficients 16"));
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
      Serial.println(F("Wrong BME280 humidity oversample"));
    }
  }

  while (1)
  {
    Serial.println(F("*** Enter sleep time in seconds that is divisible by 8:"));
    Serial.println(F("Ex. 8, 16, 32, 128"));
    while (!Serial.available())
      ;

    int sleep_time = Serial.readStringUntil('\n').toInt();

    if (sleep_time >= 8 && sleep_time <= 432000 && (sleep_time % 8) == 0)
    {
      sleep_8s_count = sleep_time / 8;
      eeprom_write_byte((uint8_t *)EEPROM_SLEEP_TIME_ADDRESS, sleep_8s_count);
      Serial.println(sleep_time);
      break;
    }
    else
    {
      Serial.println(F("Wrong sleep time"));
    }
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
      Serial.println(F("Wrong value. Enter 0 or 1"));
    }
  }

  printConfig();
}

void printConfig()
{
  Serial.println();
  Serial.print(F("Node address: "));
  Serial.print(from_node);
  Serial.println();
  Serial.print(F("Channel: "));
  Serial.print(rf_channel);
  Serial.println();
  Serial.print(F("Radio speed: "));
  Serial.print(speed_names[rf_speed]);
  Serial.println();
  Serial.print(F("Radio power: "));
  Serial.print(power_names[rf_power]);
  Serial.println();
  Serial.print(F("BME280 filter level: "));
  if (pgm_read_byte(&bme_filter_map[bme_filter]) > 0)
  {
    Serial.print(pgm_read_byte(&bme_filter_map[bme_filter]));
    Serial.print(F("x"));
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
    Serial.print(F("x"));
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
    Serial.print(F("x"));
  }
  else
  {
    Serial.print(F("Disabled"));
  }
  Serial.println();
  Serial.print(F("Sleep time seconds: "));
  Serial.print(sleep_8s_count * 8);
  Serial.println();
  Serial.print(F("Status LED: "));
  Serial.print((status_led_enabled == 1) ? F("Enabled") : F("Disabled"));
  Serial.println();
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
  int batteryPcnt = (((batteryV - battery_min_voltage) / (battery_max_voltage - battery_min_voltage)) * 100);

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

void powerDown()
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

ISR (WDT_vect)
{
	wdt_disable();
}