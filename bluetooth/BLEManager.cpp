#include "BLEManager.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

namespace {
    BLECharacteristic* pCharacteristic = nullptr;
    bool deviceConnected = false;

    /**
     * @brief BLE server callbacks to track connection state.
     */
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

void initBLE(const char* deviceName) {
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
}

void updateBLE(float ax, float ay, float az,
               float gx, float gy, float gz) {
    if (!deviceConnected || pCharacteristic == nullptr) {
        return;
    }

    String imuString =
        String(ax, 3) + "," +
        String(ay, 3) + "," +
        String(az, 3) + "," +
        String(gx, 3) + "," +
        String(gy, 3) + "," +
        String(gz, 3);

    pCharacteristic->setValue(imuString.c_str());
    pCharacteristic->notify();
}