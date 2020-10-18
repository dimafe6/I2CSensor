#include <EEPROM.h>
#include "RF24.h"
#include <SparkFunBME280.h>
#include "LowPower.h"
#include <avr/power.h>
#include "./src/GPIO.h"
//#define DEBUG

#define EEPROM_NODE_ADDRESS 20
#define EEPROM_CHANNEL_ADDRESS 21
#define EEPROM_SPEED_ADDRESS 22
#define EEPROM_POWER_ADDRESS 23
#define EEPROM_SLEEP_TIME_ADDRESS 24
#define EEPROM_ENABLE_STATUS_LED_ADDRESS 26

#define PROG_PIN 2
#define RADIO_CE 9
#define RADIO_CSN 10
#define RF_PWR PD5
#define SENSOR_PWR 4
#define CONFIG_LED A1
#define STATUS_LED A2

char const *power_names[4]{"MIN", "LOW", "HIGH", "MAX"};
char const *speed_names[3]{"1MBPS", "2MBPS", "250KBPS"};
const uint64_t pipes[5] = {0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL};

RF24 radio(RADIO_CE, RADIO_CSN);
BME280 bme280;

byte from_node = 1;
byte rf_channel = 50;
byte rf_speed = 2;
byte rf_power = 1;
byte sleep_8s_count = 1;
byte status_led_enabled = 1;

struct ExternalSensor
{
  signed int temperature;
  signed int humidity;
};

ExternalSensor data;

void setup()
{
  ADCSRA = 0;
  power_adc_disable();

  SPCR = 0;
  power_spi_disable();

  pinAsInputPullUp(PROG_PIN);
  pinAsOutput(RF_PWR);
  pinAsOutput(SENSOR_PWR);
  pinAsOutput(CONFIG_LED);
  pinAsOutput(STATUS_LED);
  digitalLow(RF_PWR);
  digitalLow(SENSOR_PWR);
  digitalLow(CONFIG_LED);
  digitalLow(STATUS_LED);

  from_node = EEPROM.read(EEPROM_NODE_ADDRESS);
  rf_channel = EEPROM.read(EEPROM_CHANNEL_ADDRESS);
  rf_speed = EEPROM.read(EEPROM_SPEED_ADDRESS);
  rf_power = EEPROM.read(EEPROM_POWER_ADDRESS);
  sleep_8s_count = EEPROM.read(EEPROM_SLEEP_TIME_ADDRESS);
  status_led_enabled = EEPROM.read(EEPROM_ENABLE_STATUS_LED_ADDRESS);

  bool configIsValid = (from_node >= 1 && from_node <= 5) &&
                       (rf_channel >= 1 && rf_channel <= 125) &&
                       (rf_speed >= 0 && rf_speed <= 2) &&
                       (rf_power >= 0 && rf_speed <= 3) &&
                       (sleep_8s_count >= 1 && sleep_8s_count <= 432000);

#ifdef DEBUG
  Serial.begin(9600);
#endif

  if (isLow(PROG_PIN) || !configIsValid)
  {
    Serial.begin(9600);
    Serial.println("");
    Serial.println("Configuration mode");

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
  Serial.println("Start");
  unsigned long startTime = millis();
#endif

  enableStatusLED();

  digitalHigh(SENSOR_PWR);

  Wire.begin();

  bme280.setI2CAddress(0x76);

  if (bme280.beginI2C() == false)
  {
#ifdef DEBUG
    Serial.println("BME280 sensor connect failed");
#endif

    data.temperature = 0;
    data.humidity = 0;

    for (byte i = 0; i <= 3; i++)
    {
      enableStatusLED();
      delay(200);
      disableStatusLED();
      delay(200);
    }

    Wire.end();
    digitalLow(SENSOR_PWR);

    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  else
  {
    bme280.setFilter(2);
    bme280.setTempOverSample(2);
    bme280.setPressureOverSample(2);
    bme280.setHumidityOverSample(2);
    bme280.setMode(MODE_FORCED);

    data.temperature = bme280.readTempC() * 100;
    data.humidity = bme280.readFloatHumidity() * 100;

    Wire.end();
    digitalLow(SENSOR_PWR);

    disableStatusLED();

    digitalHigh(RF_PWR);
    power_spi_enable();
    SPCR = 1;

    radio.begin();
    radio.powerUp();
    radio.stopListening();
    radio.setAutoAck(false);
    radio.setChannel(rf_channel);
    radio.setPayloadSize(4);
    radio.setPALevel(rf24_pa_dbm_e(rf_power));
    radio.setDataRate(rf24_datarate_e(rf_speed));
    radio.disableCRC();
    radio.openWritingPipe(pipes[from_node - 1]);

#ifdef DEBUG
    Serial.print("Humidity: ");
    Serial.print(data.humidity);
    Serial.println();

    Serial.print("Temp: ");
    Serial.print(data.temperature);
    Serial.println();
    Serial.println(radio.isChipConnected());
#endif

    radio.write(&data, sizeof(data));

    radio.powerDown();
    digitalLow(RF_PWR);

    disableStatusLED();

#ifdef DEBUG
    Serial.print("Running time(ms): ");
    Serial.println(millis() - startTime);
    Serial.println("Going to sleep");
    delay(1000); // Delay for complete Serial write
#endif
    unsigned int sleepCounter;
    for (sleepCounter = sleep_8s_count; sleepCounter > 0; sleepCounter--)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }

    ADCSRA = 0;
    power_adc_disable();

    SPCR = 0;
    power_spi_disable();
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
    Serial.println("*** Enter node address (1-5):");
    while (!Serial.available())
      ;

    from_node = Serial.readStringUntil('\n').toInt();

    if (from_node >= 1 && from_node <= 5)
    {
      EEPROM.write(EEPROM_NODE_ADDRESS, from_node);
      Serial.println(from_node);
      break;
    }
    else
    {
      Serial.println("Wrong node address");
    }
  }

  while (1)
  {
    Serial.println("*** Enter node channel id (1-125):");
    while (!Serial.available())
      ;

    rf_channel = Serial.readStringUntil('\n').toInt();

    if (rf_channel >= 1 && rf_channel <= 125)
    {
      EEPROM.write(EEPROM_CHANNEL_ADDRESS, rf_channel);
      Serial.println(rf_channel);
      break;
    }
    else
    {
      Serial.println("Wrong channel id");
    }
  }

  while (1)
  {
    Serial.println("*** Select radio speed (0-2):");
    Serial.println("0 - 1MBPS\r\n1 - 2MBPS\r\n2 - 250KBPS");
    while (!Serial.available())
      ;

    rf_speed = Serial.readStringUntil('\n').toInt();

    if (rf_speed >= 0 && rf_speed <= 2)
    {
      EEPROM.write(EEPROM_SPEED_ADDRESS, rf_speed);
      Serial.println(rf_speed);
      break;
    }
    else
    {
      Serial.println("Wrong radio speed");
    }
  }

  while (1)
  {
    Serial.println("*** Select radio power (0-3):");
    Serial.println("0 - MIN\r\n1 - LOW\r\n2 - HIGH\r\n3 - MAX");
    while (!Serial.available())
      ;

    rf_power = Serial.readStringUntil('\n').toInt();

    if (rf_power >= 0 && rf_speed <= 3)
    {
      EEPROM.write(EEPROM_POWER_ADDRESS, rf_power);
      Serial.println(rf_power);
      break;
    }
    else
    {
      Serial.println("Wrong radio power");
    }
  }

  while (1)
  {
    Serial.println("*** Enter sleep time in seconds that is divisible by 8:");
    Serial.println("Ex. 8, 16, 32, 128");
    while (!Serial.available())
      ;

    int sleep_time = Serial.readStringUntil('\n').toInt();

    if (sleep_time >= 8 && sleep_time <= 432000 && (sleep_time % 8) == 0)
    {
      sleep_8s_count = sleep_time / 8;
      EEPROM.write(EEPROM_SLEEP_TIME_ADDRESS, sleep_8s_count);
      Serial.println(sleep_time);
      break;
    }
    else
    {
      Serial.println("Wrong sleep time");
    }
  }

  while (1)
  {
    Serial.println("*** Enable status LED?");
    Serial.println("1 - Enable\r\n0 - Disable");
    while (!Serial.available())
      ;

    status_led_enabled = Serial.readStringUntil('\n').toInt();

    if (status_led_enabled >= 0 && status_led_enabled < 2)
    {
      EEPROM.write(EEPROM_ENABLE_STATUS_LED_ADDRESS, status_led_enabled);
      Serial.println((status_led_enabled == 1) ? "Enabled" : "Disabled");
      break;
    }
    else
    {
      Serial.println("Wrong value. Enter 0 or 1");
    }
  }

  printConfig();
}

void printConfig()
{
  Serial.println();
  Serial.print("Node address: ");
  Serial.print(from_node);
  Serial.println();
  Serial.print("Channel: ");
  Serial.print(rf_channel);
  Serial.println();
  Serial.print("Radio speed: ");
  Serial.print(speed_names[rf_speed]);
  Serial.println();
  Serial.print("Radio power: ");
  Serial.print(power_names[rf_power]);
  Serial.println();
  Serial.print("Sleep time seconds: ");
  Serial.print(sleep_8s_count * 8);
  Serial.println();
  Serial.print("Status LED: ");
  Serial.print((status_led_enabled == 1) ? "Enabled" : "Disabled");
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
