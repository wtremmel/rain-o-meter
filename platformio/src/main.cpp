#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <ArduinoLog.h>
#include "CubeCell_NeoPixel.h"
#include "innerWdt.h"

#include "cubecell.h"

#include <CayenneLPP.h>
CayenneLPP lpp(51);

// Sensor Libraries


#define BATTERY_RECHARGABLE 1
#define HAS_RGB 1

#define SLEEP_INTERVAL 60*1000*4 // 4 Minutes
#define MAX_CYCLE SLEEP_INTERVAL
#define MIN_CYCLE 10
#define MAX_VOLTAGE 4100
#define MIN_VOLTAGE 3700
#define MEASUREMENT_INTERVAL 60*1000 // 1 Minutes
#define LOGLEVEL LOG_LEVEL_VERBOSE
// #define LOGLEVEL LOG_LEVEL_SILENT

uint32_t sleep_interval = SLEEP_INTERVAL,
  measurement_interval = MEASUREMENT_INTERVAL,
  measurement_start,
  measurement_stop,
  measurement_counter;


#define S_STARTING 1
#define S_MEASUREMENT 2
#define S_COMPLETE 3
#define S_WAIT 4

uint8_t state = S_STARTING, prev_reading = 0;
#define MEASUREMENT_PIN GPIO1

// #define LOGLEVEL LOG_LEVEL_VERBOSE


// Global Objects
#if HAS_RGB
CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);
#endif


bool setup_complete = false;
bool pixels_initalized = false;
bool drain_battery = false;
bool voltage_found = true;


uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr;
/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
#if BATTERY_RECHARGABLE
uint32_t appTxDutyCycle = SLEEP_INTERVAL;
bool variableDutyCycle = true;
#else
uint32_t appTxDutyCycle = SLEEP_INTERVAL;
bool variableDutyCycle = false;
#endif
/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

uint8_t ledr = 0, ledg = 0, ledb = 0;
bool ledon = false;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;


void print_counters() {
  Log.verbose(F("sleep_interval = %u"),sleep_interval);
  Log.verbose(F("measurement_interval = %u"),measurement_interval);
  Log.verbose(F("measurement_start = %u"),measurement_start);
  Log.verbose(F("measurement_stop = %u"),measurement_stop);
  Log.verbose(F("measurement_counter = %u"),measurement_counter);
  Log.verbose(F("millis() = %u"),millis());
}

// external power functions
void vext_power(bool on) {
  if (on) {
    digitalWrite(Vext,LOW);
  } else {
    digitalWrite(Vext,HIGH);
  }
}

void set_led(uint8_t r, uint8_t g, uint8_t b) {
  // switch on power
#if HAS_RGB
  vext_power(true);

  // Log.verbose(F("set_led(%d,%d,%d)"),r,g,b);
  if (!pixels_initalized){
    pixels.begin();
    pixels_initalized = true;
  }

  if (r == 0 && g == 0 && b == 0) {
    pixels.clear();
    pixels.show();
  } else {
    pixels.setPixelColor(0, pixels.Color(r,g,b));
    pixels.show();
    // delay(10*1000);
  }
#endif
}



// Battery voltage
void read_voltage() {
  uint16_t v = getBatteryVoltage();
  lpp.addAnalogInput(5,(float)v / 1000.0);
  Log.verbose(F("Voltage: %d"),v);

  long int cycle =
  (
    (
      (long)MIN_CYCLE - (long)MAX_CYCLE
    )
      / ((long)MAX_VOLTAGE-(long)MIN_VOLTAGE)
  ) *
  (v - (long)MIN_VOLTAGE) +
  (long)MAX_CYCLE;

  if (cycle < MIN_CYCLE)
    cycle = MIN_CYCLE;
  if (cycle > MAX_CYCLE)
    cycle = MAX_CYCLE;
  if (variableDutyCycle) {
    sleep_interval = cycle;
    appTxDutyCycle = cycle;
  }

}

// Sensor routines
void read_sensors() {
  lpp.reset();

  // switch on power
  vext_power(true);
  set_led(ledr,ledg,ledb);

  delay(100);

  if (voltage_found) {
    read_voltage();
  }
  if (state == S_COMPLETE) {
    lpp.addLuminosity(9,measurement_counter);
    Log.verbose(F("Wind = %d"), measurement_counter);
  }

}



void setup_serial() {
  Serial.begin(115200);
#if DEBUG
  while (!Serial);
#endif
}


// Logging helper routines
void printTimestamp(Print* _logOutput, int logLevel) {
  static char c[12];
  // sprintf(c, "%l ", TimerGetCurrentTime());
  sprintf(c, "%d ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput, int logLevel) {
  _logOutput->print('\n');
}

void setup_logging() {
  Log.begin(LOGLEVEL, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

void setup_lora() {
  Log.verbose(F("setup_lora: start"));
  deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void setup_check_voltage() {
  // Check if voltage is above restart_voltage
  uint16_t v = getBatteryVoltage();
  Log.verbose(F("Voltage: %d"),v);
}

void setup_chipid() {
  uint64_t chipID=getID();
  Log.notice(F("Chip ID = %X%x"),
    (uint32_t)(chipID>>32),(uint32_t)chipID);
}

void set_default_timers() {
  sleep_interval = SLEEP_INTERVAL;
  appTxDutyCycle = SLEEP_INTERVAL;
  measurement_interval = MEASUREMENT_INTERVAL;
  variableDutyCycle = true;
}

void setup() {
  // Turn on watchdog
  innerWdtEnable(true);

  setup_serial();
  delay(5000);
  setup_logging();
  Log.verbose(F("setup(): Logging started"));
  setup_chipid();
  setup_check_voltage();

  // Turn on power for devices
  pinMode(Vext,OUTPUT);
  vext_power(true);
  set_led(ledr,ledg,ledb);

  setup_lora();

  pinMode(MEASUREMENT_PIN,INPUT_PULLUP);
}

static void prepareTxFrame( ) {
  read_sensors();

  appDataSize = lpp.getSize();
  memcpy(appData,lpp.getBuffer(),appDataSize);
}

// -------------- Command Processing -----------------
void process_system_led_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length LED command"));
    return;
  } else {
    Log.verbose(F("Processing LED command"));
  }

  switch (buffer[0]) {
    case 0x00:
      ledr = 0;
      ledg = 0;
      ledb = 0;
      set_led(0,0,0);
      break;
    case 0x01:
      ledr = 255;
      ledg = 255;
      ledb = 255;
      set_led(255,255,255);
      break;
#if HAS_RGB
    case 0x02:
      if (len == 4) {
        // do rgb magic
        ledr = buffer[1];
        ledg = buffer[2];
        ledb = buffer[3];
        set_led(ledr,ledg,ledb);
      } else {
        Log.error(F("Missing RGB values for LED. Len = %d"),len);
      }
      break;
#endif
    default:
      Log.error(F("Unknown LED command %d"), buffer[0]);
      break;
  }
}

void process_system_power_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length power command"));
  } else {
    Log.verbose(F("Processing power command"));
  }

  switch (buffer[0]) {
    case 0x01:
      drain_battery = false;
      Log.verbose(F("Power save to default"));
      break;
#if BATTERY_RECHARGABLE
    case 0xff:
      drain_battery = true;
      Log.verbose(F("Drain battery on"));
      break;
#endif
    default:
      Log.error(F("Unknown power command %d"),buffer[0]);
      break;
  }
}



void process_system_delay_command(unsigned char len, unsigned char *buffer) {
  if (len != 1) {
    Log.error(F("Len of delay command != 1"));
  } else {
    Log.verbose(F("Processing delay command"));
  }

}

void process_system_timer_command(unsigned char len, unsigned char *buffer) {
  if (len <= 1) {
    Log.error(F("Len of timer command <= 1"));
  } else {
    Log.verbose(F("Processing timer command"));
  }

  switch(buffer[0]){
    case 0x00:
      set_default_timers();
      break;
    case 0x03:
      if (buffer[1] == 0) {
        // dynamic sleep time
        variableDutyCycle = true;
      } else if (buffer[1] < 10) {
        sleep_interval = 60*1000*buffer[1];
        appTxDutyCycle = sleep_interval;
        variableDutyCycle = false;
      } else {
        // illegal value
        set_default_timers();
      }
      break;
    default:
      Log.error(F("Unknown timer command %X"),buffer[0]);
  }
}

void process_system_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length system command"));
    return;
  } else {
    Log.verbose(F("Processing system command"));
  }
  switch (buffer[0]) {
    case 0x01:
      process_system_power_command(len-1,buffer+1);
      break;
    case 0x02:
      process_system_delay_command(len-1,buffer+1);
      break;
    case 0x03:
      process_system_led_command(len-1,buffer+1);
      break;
    case 0x08:
      process_system_timer_command(len-1,buffer+1);
      break;
    case 0xff:
      // Reboots
      Log.notice(F("Executing reboot command"));
      delay(100);
      HW_Reset(0);
    default:
      Log.error(F("Unknown system command %d"),buffer[0]);
      break;
  }
}

void process_received_lora(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  Log.verbose(F("Processing %d bytes of received data"),len);
  switch (buffer[0]) {
    case 0:
      process_system_command(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown command %d"),buffer[0]);
      break;
  }
}


void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Log.verbose(F("+REV DATA:%s,RXSIZE %d,PORT %d\r\n"),
    mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",
    mcpsIndication->BufferSize,
    mcpsIndication->Port);
  process_received_lora(mcpsIndication->BufferSize, mcpsIndication->Buffer);
}



void loop() {
  setup_complete = true;

  if (state == S_STARTING) {
    // we just started. start measurements
    vext_power(true);
    measurement_start = millis();
    measurement_stop = measurement_start + MEASUREMENT_INTERVAL;
    measurement_counter = 0;
    state = S_MEASUREMENT;
    Log.verbose(F("S: S_STARTING"));
  }

  if (state == S_MEASUREMENT) {
    // count pulses
    uint8_t v;
    v = digitalRead(MEASUREMENT_PIN);
    if (v != prev_reading) {
      if (v == 0) {
        measurement_counter++;
        set_led(1, 0, 0);
      } else {
        set_led(0, 0, 0);
      }
      prev_reading = v;
    }

    if (millis() > measurement_stop) {
      state = S_COMPLETE; // add after debugging
    }
  }

  if (state == S_COMPLETE) {
  switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
			LoRaWAN.generateDeveuiByChipID();
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
      Log.verbose(F("S: S_COMPLETE. counter = %d"),measurement_counter);
      prepareTxFrame();
      LoRaWAN.send();
		  deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle;
      Log.verbose(F("DEVICE_STATE_CYCLE: Duty cycle: %d s"),int(appTxDutyCycle / 1000));
      LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
      // switch off power
      if (!drain_battery)
        vext_power(false);
      // Log.verbose(F("Sleeping - txDutyCycleTime = %d"),txDutyCycleTime);
      // delay(10);
		  LoRaWAN.sleep();
      if (millis() > (measurement_stop + sleep_interval)) {
        state = S_STARTING;
      }
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}
}
