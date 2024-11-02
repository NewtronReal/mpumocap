#include "I2Cdev.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define SSID "VDMAEQ720VD" //model number of vethram VD stands for Vethram Device
#define PASSWORD "something"
#define UDP_PORT 4444
#define SLAVE_NO 0x02
 

MPU6050 mpu;
WiFiUDP UDP;

#define OUTPUT_TEAPOT


bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint8_t teapotPacket[14] = { '$', SLAVE_NO, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(0,2);
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); 

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

 
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));


        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {

        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    WiFi.begin(SSID,PASSWORD);
    while(WiFi.status()!=WL_CONNECTED){
      delay(100);
      Serial.print(".");
    }
    Serial.println();
    Serial.println("Connected IP address: ");
    Serial.println(WiFi.localIP());
    UDP.begin(UDP_PORT);
}




void loop() {
    if (!dmpReady) {Serial.println("not ready");return;}
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    
        #ifdef OUTPUT_TEAPOT
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            Serial.println("sending..");
            UDP.beginPacket("192.168.4.1",4444);
            UDP.write(teapotPacket,14);
            UDP.endPacket();
            teapotPacket[11]++; 
        #endif

    }
}
