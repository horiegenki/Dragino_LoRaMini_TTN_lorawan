/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Changed 2017.11.01 OpenWave inc, 
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 * 
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * Required Library: 
 *    * https://github.com/matthijskooijman/arduino-lmic 
 *    * https://github.com/adafruit/DHT-sensor-library
 *    * https://github.com/adafruit/Adafruit_Sensor
 * 
 * Require Hardware:
 *    * LoRa Shield + Arduino
 *    * LoRa GPS Shield + Arduino 
 *    * LoRa Mini etc. 
 *    
 *    このサンプルは、The Things NetworkにABPで、DHT11の
 *    温度、湿度のデータを送信します。
 *    
 *    2017.11.01 株式会社オープンウェーブ
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"

#define dht_dpin A0     // Use A0 pin as Data pin for DHT11. 
#define DHTTYPE DHT11   // DHT 11 

/*
 * 以下の、デバイスEUI、アプリケーションEUI、アプリケーションセッションキー
 * を、The Things Networkで取得した値に変更してください。
 */
// デバイスEUI(リトルエンディアン)
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// アプリケーションEUI(リトルエンディアン)
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//  アプリケーションキー
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// 温度・湿度データを管理
static uint8_t mydata[2];

// 送信待ち時間
const unsigned TX_INTERVAL = 1;

static osjob_t initjob,sendjob,blinkjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

DHT dht(dht_dpin, DHTTYPE);

void do_send(osjob_t* j){

    uint32_t h = dht.readHumidity();
    uint32_t t = dht.readTemperature();

    mydata[0] = h;
    mydata[1] = t;
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println("Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void setup() {

    memset(mydata, 0x00, 4);
    
    // Serial.begin(9600);

    while(!Serial);
    
    Serial.println("Starting");
    
    delay(1000);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // TTN uses SF9 for its RX2 window.
    //LMIC.dn2Dr = AS923_DR_SF9;
    Serial.println(F("LMIC SET DN"));
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(AS923_DR_SF10,13);

    Serial.println(F("LMIC SET TX"));

    dht.begin();
    
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

