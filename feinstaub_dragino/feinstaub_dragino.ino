

// MIT License
// Code for measuring air quality with an Arduino and a Dragino sensor.
// The data is send via loraWAN
// The values are measured for a project of Klimainitiative Dorsten
// www.klimainitiative-dorsten.de

// Based on the tutorial of Stefan Schultheis
// https://stefan.schultheis.at/2017/lora-sensor-arduino-lora-shield/
//
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// Based on the adaptions of: Andreas Spiess

#include <lmic.h>
#include <hal/hal.h>


/////////////////////////////////////
//Ergaenzungen fuer Feinstaubsensor

#include <SoftwareSerial.h>
SoftwareSerial pmsSerial(2, 3);

//#include <credentials.h>

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;




boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}



//Ende Ergaenzungen fuer Feinstaubsensor
//////////////////////////////////////







#ifdef CREDENTIALS
static const u1_t NWKSKEY[16] = NWKSKEY1;
static const u1_t APPSKEY[16] = APPSKEY1;
static const u4_t DEVADDR = DEVADDR1;
#else
static const u1_t NWKSKEY[16] = { 0x79, 0x7A, 0x61, 0x16, 0x42, 0xDC, 0xEB, 0x22, 0x5D, 0xD5, 0x3D, 0xA1, 0xD2, 0xB5, 0x0E, 0x23 };
static const u1_t APPSKEY[16] = { 0x8A, 0x3B, 0xC6, 0x43, 0x01, 0x17, 0xA4, 0xC0, 0x4B, 0x0F, 0x09, 0x44, 0xFD, 0xCD, 0x8C, 0xAD };
static const u4_t DEVADDR = 0x260119E2;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;

// Pin mapping Dragino Shield
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};
void onEvent (ev_t ev) {
  if (ev == EV_TXCOMPLETE) {
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
  }
}

void do_send(osjob_t* j) {
  // Payload to send (uplink)

  //Feinstaubsensor auslesen
  if (readPMSdata(&pmsSerial)) { 
      static uint16_t *message; //uint16_t
      message = calloc(1, sizeof(uint16_t) );      
      *message= data.pm10_standard;
      //*(message+1)= data.pm25_standard;
      //*(message+2)= data.pm25_standard;

      // Check if there is not a current TX/RX job running
      if (LMIC.opmode & OP_TXRXPEND) {
          Serial.println(F("OP_TXRXPEND, not sending"));
      } else {
          // Prepare upstream data transmission at the next possible time.
          LMIC_setTxData2(1, *message, sizeof(message) - 1, 0);
          Serial.println(F("Sending uplink packet..."));
      }
      // Next TX is scheduled after TX_COMPLETE event.
    
  }

}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting..."));

  //FeinstaubsensorSetup
  pmsSerial.begin(9600);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF12, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
