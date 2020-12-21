#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include "SparkFun_Ublox_Arduino_Library.h" 
#include "axp20x.h"

#include "BluetoothSerial.h"

#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define BATT_MILLIVOLTS_EMPTY 3500
#define BATT_MILLIVOLTS_FULL 4100

BluetoothSerial SerialBT;

LSM9DS1 imu;
SFE_UBLOX_GPS ublox;
HardwareSerial GPSSerial1(1);
AXP20X_Class axp;

bool deviceConnected = false;
bool oldDeviceConnected = false;


bool hasIMU = false;
uint32_t lastIMU;
uint32_t periodIMU = 200;

uint32_t lastBatteryCheck;
uint32_t periodBatteryCheck = 2000;

void setup() {
  Serial.begin(9600);

  // setup i2c bus
  if(!Wire.begin(21, 22))
  {
    Serial.println("not connected to I2C");
  }

  // setup PMU
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS\n");

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300); // for the OLED power
    axp.setChargeControlCur(AXP1XX_CHARGE_CUR_1320MA);
    axp.setChgLEDMode(AXP20X_LED_OFF);

    Serial.println(axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
    Serial.println(axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
    Serial.println(axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
    Serial.println(axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
    Serial.println(axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
    Serial.println(axp.isExtenEnable() ? "ENABLE" : "DISABLE");

    if(axp.isBatteryConnect()) {
      axp.setChargingTargetVoltage(AXP202_TARGET_VOL_4_15V);
      axp.enableChargeing(true);
    }
  }

  bool gpsConnected = false;

  // setup gps
    GPSSerial1.begin(9600, SERIAL_8N1, 34, 12);

    if(GPSSerial1) {
      gpsConnected = ublox.begin(GPSSerial1);

      if(!gpsConnected) {
        delay(1100);
        Serial.println("RETRY CONNECT");
        gpsConnected = ublox.begin(GPSSerial1);
      }
    }

  if(gpsConnected) {
    ublox.factoryReset();
    delay(3000);

    Serial.println("SWITCH BAUD");
    ublox.setSerialRate(115200, COM_PORT_UART1);
    
    GPSSerial1.flush();
    delay(1000);

    GPSSerial1.updateBaudRate(115200);
  }
  else {
    GPSSerial1.updateBaudRate(115200);

    if(GPSSerial1) {
      gpsConnected = ublox.begin(GPSSerial1);

      if(!gpsConnected) {
        gpsConnected = ublox.begin(GPSSerial1);
      }
    }
  }

    if(gpsConnected) {
      ublox.setNavigationFrequency(1);

      ublox.setUART1Output(COM_TYPE_NMEA); 

      ublox.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
      ublox.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);

      ublox.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); //Only leaving GGA enabled at current navigation rate
      ublox.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);

  } else {
    Serial.println("GPS NOT CONNECTED");
  }

  hasIMU = imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire) != 0;
  SerialBT.begin("RC_TBEAM");
}

void loop() {
  uint32_t now = millis();

  if (periodBatteryCheck && (now - lastBatteryCheck) >= periodBatteryCheck) {

    if(!deviceConnected) {
      deviceConnected = SerialBT.hasClient();

      if(deviceConnected) {
        Serial.println("BT CONNECTED");
        ublox.setNavigationFrequency(15);
      }
    } else {
      deviceConnected = SerialBT.hasClient();

      if(!deviceConnected) {
        Serial.println("BT DISCONNECTED");
        ublox.setNavigationFrequency(1);
      }
    }

    lastBatteryCheck = now;

    // read power
    bool hasBattery = axp.isBatteryConnect();
    int batteryVoltageMv = 0;
    uint8_t batteryChargePercent = 0;

    if(hasBattery) {
      batteryVoltageMv = axp.getBattVoltage();
      batteryChargePercent = axp.getBattPercentage();
      bool isCharging = axp.isChargeing();
      float acVoltage = axp.getAcinVoltage();

      Serial.printf("BATTERY: %d , %d mv \n", batteryChargePercent, batteryVoltageMv);
      Serial.printf("AC Voltage: %f \n", acVoltage);

      // if we arent charging and battery is low voltage, flash LED and shut down
      if(!isCharging && batteryVoltageMv <= (BATT_MILLIVOLTS_EMPTY + 50)) {
            Serial.println("WARNING LOW BATTERY - SHUTTING DOWN");
            axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
            delay(5000);
            axp.shutdown();
      }

      if(isCharging) {
        axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
      } else {
        axp.setChgLEDMode(AXP20X_LED_OFF);
      }

      if(!isCharging && acVoltage > 3.0f) {
        axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
      }
    }
}

  // wait for bluetooth connection
  if(deviceConnected) {

    uint8_t rc_data[156];
    uint8_t rc3_data[100];
    uint8_t last_time[8];
    int counter = 0;

    bool beginPacket = false;
    bool endPacket = false;

    while(GPSSerial1.available() && !endPacket && counter < 156) {

      uint8_t data = GPSSerial1.read();

      if(!beginPacket) {
        if(data == '$') {
          beginPacket = true;
        }
      }

      if(beginPacket) {
        if(data == '\n') {
          endPacket = true;
        }
      }

      if(beginPacket) {
        rc_data[counter] = data;
        counter++;
      }
    }

    //send to BLE
    if(counter != 0) {
      SerialBT.write(rc_data, counter);

      // if $GPGGA packet, store time
      if(rc_data[4] == 'G' && rc_data[5] == 'A') {
        // char 7 - 15 should be HHMMSS.ss
        for(int i=0; i<8; i++) {
          last_time[i] = rc_data[7 + i];
        }
      }
    }

    if (hasIMU && periodIMU && (now - lastIMU) >= periodIMU) {

      lastIMU = now;
      uint8_t imu_data[64];

      if ( imu.gyroAvailable() )
      {
        imu.readGyro();
        Serial.printf("GX: %f, GY: %f, GZ: %f \n", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
      }
      if ( imu.accelAvailable() )
      {
        imu.readAccel();
        Serial.printf("AX: %d AY: %d AZ: %d \n", imu.ax, imu.ay, imu.az);
      }

      // construct rc3 packet
      // $RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],[rpm/d1],[d2],[a1],[a2],[a3],[a4],[a5],[a6],[a7],[a8],[a9],[a10],[a11],[a12],[a13],[a14],[a15]*checksum

      // send to bt
    }
  } else {
    // no bt connected, sleep
    delay(1000);
  }
}