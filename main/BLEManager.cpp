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

void updateBLEQuats(float uq0, float uq1, float uq2, float uq3,
                    float fq0, float fq1, float fq2, float fq3) {
    if (!deviceConnected || pCharacteristic == nullptr) return;

    String msg =
        String(uq0, 6) + "," +
        String(uq1, 6) + "," +
        String(uq2, 6) + "," +
        String(uq3, 6) + "," +
        String(fq0, 6) + "," +
        String(fq1, 6) + "," +
        String(fq2, 6) + "," +
        String(fq3, 6);

    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
}