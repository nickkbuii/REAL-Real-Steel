#include "BLEManager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

namespace {
    BLECharacteristic* pCharacteristic = nullptr;
    bool deviceConnected = false;

    class ServerCallbacks : public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
      }
      void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        pServer->startAdvertising();
      }
    };
}

bool initBLE(const char* deviceName) {
    BLEDevice::init(deviceName);

    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService("180A");

    pCharacteristic = pService->createCharacteristic(
        "2A57",
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setValue("Hello!");

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("180A");
    BLEDevice::startAdvertising();
    return true
}

void updateBLEIMU(float ax1, float ay1, float az1,
                  float gx1, float gy1, float gz1,
                  float ax2, float ay2, float az2,
                  float gx2, float gy2, float gz2) {
    if (!deviceConnected || pCharacteristic == nullptr) return;

    String msg =
        String(ax1, 6) + "," +
        String(ay1, 6) + "," +
        String(az1, 6) + "," +
        String(gx1, 6) + "," +
        String(gy1, 6) + "," +
        String(gz1, 6) + "," +
        String(ax2, 6) + "," +
        String(ay2, 6) + "," +
        String(az2, 6) + "," +
        String(gx2, 6) + "," +
        String(gy2, 6) + "," +
        String(gz2, 6);

    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
}