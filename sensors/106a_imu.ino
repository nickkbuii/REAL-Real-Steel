#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

MPU6050 mpu;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float ax, ay, az, gx, gy, gz = 0.0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Serial.println("Initializing MPU6050...");
  // mpu.initialize();

  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed!");
  //   while (1);
  // }
  // Serial.println("MPU6050 connected.");

  BLEDevice::init("NanoESP32");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService("180A");

  pCharacteristic = pService->createCharacteristic(
      "2A57",
      BLECharacteristic::PROPERTY_READ   |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Hello!");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("180A");
  BLEDevice::startAdvertising();

  Serial.println("BLE is running...");
}

void loop() {
  // int16_t ax, ay, az, gx, gy, gz;

  // mpu.getAcceleration(&ax, &ay, &az);
  // mpu.getRotation(&gx, &gy, &gz);

  // // Convert raw values to G and deg/s
  // float ax_g = ax / 16384.0;
  // float ay_g = ay / 16384.0;
  // float az_g = az / 16384.0;

  // float gx_dps = gx / 131.0;
  // float gy_dps = gy / 131.0;
  // float gz_dps = gz / 131.0;

  if (deviceConnected) {
      String imuString = 
          String(ax, 3) + "," +
          String(ay, 3) + "," +
          String(az, 3) + "," +
          String(gx, 3) + "," +
          String(gy, 3) + "," +
          String(gz, 3);

      // String imuString =
      //   String(ax_g, 3) + "," +
      //   String(ay_g, 3) + "," +
      //   String(az_g, 3) + "," +
      //   String(gx_dps, 3) + "," +
      //   String(gy_dps, 3) + "," +
      //   String(gz_dps, 3);

      pCharacteristic->setValue(imuString.c_str());
      pCharacteristic->notify();
      delay(20);
  }
}
