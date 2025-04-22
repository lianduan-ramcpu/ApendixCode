#include <Arduino_LSM6DS3.h> // Built-in IMU library
#include <ArduinoBLE.h>      // BLE library

#define ALPHA 0.98  // Complementary filter weight (Gyro: 98%, Accel: 2%)
#define BUFFER_SIZE 10  // Number of readings to average
#define TILT_THRESHOLD 30  // Threshold angle in degrees
#define DURATION_THRESHOLD 10000 // Time in ms (10 seconds)

// Kalman filter variables
float kalmanAngle = 0;
float kalmanBias = 0;
float kalmanP[2][2] = {{1, 0}, {0, 1}};
float kalmanQ_angle = 0.001;
float kalmanQ_bias = 0.003;
float kalmanR_measure = 0.03;

float tiltOffset = 0;
unsigned long tiltStartTime = 0;  
bool notificationSent = false;
bool secondWarningSent = false;
bool finalWarningSent = false;
// Pin for LED
const int ledPin = 13;
BLEService sensorService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic sensorDataCharacteristic(
    "abcdef01-1234-5678-1234-56789abcdef0",
    BLERead | BLENotify,
    64);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  tiltOffset = calibrateTilt();

  BLE.setDeviceName("Nano33IoT");
  BLE.setLocalName("Nano33IoT");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorDataCharacteristic);
  BLE.addService(sensorService);
  BLE.advertise();
  pinMode(ledPin, OUTPUT);// set LED as output
  Serial.println("BLE advertising started...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("Connected to central");

    while (central.connected()) {
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        float ax, ay, az, gx, gy, gz;
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        float gravityMag = sqrt(ax * ax + ay * ay + az * az);
        float tiltAngle = acos(az / gravityMag) * 180.0 / PI;
        tiltAngle -= tiltOffset;
        tiltAngle = kalmanFilter(tiltAngle, gz);

        if (tiltAngle < 0) tiltAngle = 0;
        // Ignore small fluctuations
        if (abs(tiltAngle) < 1) tiltAngle = 0;
        tiltAngle = round(tiltAngle * 10) / 10.0;  // Round to 1 decimal place

        if (tiltAngle >= TILT_THRESHOLD) {
            if (tiltStartTime == 0) {
                tiltStartTime = millis();
            } else {
                unsigned long elapsedTime = millis() - tiltStartTime;

                if (elapsedTime >= DURATION_THRESHOLD && !notificationSent) {
                    sendPostureNotification("Warning: Poor posture detected!");
                    notificationSent = true;
                   // Turn on LED when bad posture is detected （suppose to be vibration motor)
                digitalWrite(ledPin, HIGH);  // Turn on LED
                } 

                else if (elapsedTime >= 30000 && !secondWarningSent) {  //  30s warning
                    sendPostureNotification("Fix your posture as soon as possible");
                    secondWarningSent = true;
                    digitalWrite(ledPin, HIGH);  // Turn on LED
                } 

                else if (elapsedTime >= 60000 && !finalWarningSent) {  // 60s warning
                    sendPostureNotification("Fix your posture now!");
                    finalWarningSent = true;
                    digitalWrite(ledPin, HIGH);  // Turn on LED
                }
            }
        } else {  
            tiltStartTime = 0;
            notificationSent = false;
            secondWarningSent = false;
            finalWarningSent = false;
            // Turn off LED when posture is corrected
            digitalWrite(ledPin, LOW);  // Turn off LED
        }
        delay(500);
      }
    }

    Serial.println("Disconnected from central");
  }
}

// Function to send BLE posture warnings
void sendPostureNotification(String message) {
  Serial.println(message);
  sensorDataCharacteristic.writeValue(message.c_str());
}

// Kalman filter function
float kalmanFilter(float newAngle, float newRate) {
    float dt = 0.01;

    kalmanAngle += dt * (newRate - kalmanBias);
    kalmanP[0][0] += dt * (dt * kalmanP[1][1] - kalmanP[0][1] - kalmanP[1][0] + kalmanQ_angle);
    kalmanP[0][1] -= dt * kalmanP[1][1];
    kalmanP[1][0] -= dt * kalmanP[1][1];
    kalmanP[1][1] += kalmanQ_bias * dt;

    float y = newAngle - kalmanAngle;
    float S = kalmanP[0][0] + kalmanR_measure;
    float K[2];
    K[0] = kalmanP[0][0] / S;
    K[1] = kalmanP[1][0] / S;

    kalmanAngle += K[0] * y;
    kalmanBias += K[1] * y;

    kalmanP[0][0] -= K[0] * kalmanP[0][0];
    kalmanP[0][1] -= K[0] * kalmanP[0][1];
    kalmanP[1][0] -= K[1] * kalmanP[0][0];
    kalmanP[1][1] -= K[1] * kalmanP[0][1];

    return kalmanAngle;
}

// Function to calibrate tilt at startup
float calibrateTilt() {
  float ax, ay, az;
  Serial.println("Calibrating tilt offset...");

  while (!IMU.accelerationAvailable());

  IMU.readAcceleration(ax, ay, az);

  float gravityMag = sqrt(ax * ax + ay * ay + az * az);
  float initialTilt = acos(az / gravityMag) * 180.0 / PI;

  Serial.print("Tilt Offset Set to: ");
  Serial.println(initialTilt);

  return initialTilt;
}
