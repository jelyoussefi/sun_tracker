#include <Wire.h>
#include <ds3231.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Ephem_Soleil.h>

// Constants
#define INTERRUPT_PIN 2
#define LED_PIN 13
#define ADDRESS 0x1E
const int LATITUDE = 33.000000, LONGITUDE = -7.616700;

// Global variables
MPU6050 mpu(0x69);
volatile bool mpuInterrupt = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float panelHeight;
struct ts currentTime;
String date, formattedDate, dayString, monthString;

// Function prototypes
void dmpDataReady();
void setup_MPU6050();
void displayTime();
void compass();
void calculateSunPosition();
void calculateSunriseSunset();

void setup() {
    Serial.begin(9600);
    Wire.begin();

    pinMode(LED_PIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT);

    // Initialize DS3231 RTC
    DS3231_init(DS3231_INTCN);

    // Initialize MPU6050
    setup_MPU6050();

    // Initial sun position setup
    displayTime();
    calculateSunriseSunset();
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= packetSize) {
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        panelHeight = -ypr[1] * 180 / M_PI;
        Serial.println(panelHeight);
    }

    displayTime();
    calculateSunPosition();
    delay(1000);
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup_MPU6050() {
    Wire.setClock(400000);
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void displayTime() {
    DS3231_get(&currentTime);
    if (currentTime.mday < 10) dayString = "0" + String(currentTime.mday);
    else dayString = String(currentTime.mday);

    if (currentTime.mon < 10) monthString = "0" + String(currentTime.mon);
    else monthString = String(currentTime.mon);

    formattedDate = dayString + "/" + monthString + "/" + String(currentTime.year);
    date = String(currentTime.hour) + ":" + String(currentTime.min) + ":" + String(currentTime.sec) + " " + formattedDate;

    Serial.println(date);
}

void calculateSunPosition() {
    double elevation, azimuth;
    posSoleil(currentTime.year, currentTime.mon, currentTime.mday, currentTime.hour, currentTime.min, currentTime.sec, 0, LATITUDE, LONGITUDE, &elevation, &azimuth);
    Serial.print("Elevation: ");
    Serial.print(elevation);
    Serial.print(" Azimuth: ");
    Serial.println(azimuth);
}

void calculateSunriseSunset() {
    double sunriseElevation, sunriseAzimuth, sunsetElevation, sunsetAzimuth;
    String sunriseTime, sunsetTime;
    lmvSoleil(formattedDate, 0, 0, LATITUDE, LONGITUDE, &sunriseTime, nullptr, &sunsetTime, 30);
    posSoleil(formattedDate + " " + sunriseTime, 0, LATITUDE, LONGITUDE, &sunriseElevation, &sunriseAzimuth);
    posSoleil(formattedDate + " " + sunsetTime, 0, LATITUDE, LONGITUDE, &sunsetElevation, &sunsetAzimuth);
    Serial.print("Sunrise Elevation: ");
    Serial.print(sunriseElevation);
    Serial.print(" Azimuth: ");
    Serial.println(sunriseAzimuth);
    Serial.print("Sunset Elevation: ");
    Serial.print(sunsetElevation);
    Serial.print(" Azimuth: ");
    Serial.println(sunsetAzimuth);
}
