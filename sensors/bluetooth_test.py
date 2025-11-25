import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "180A"
CHAR_UUID = "2A57"  # characteristic that sends millis()

async def notification_handler(sender, data):
    print(f"Notification from {sender}: {data.decode('utf-8')}")
    
    imu_text = data.decode('utf-8')
    ax, ay, az, gx, gy, gz = map(float, imu_text.split(","))
    print("Accel:", ax, ay, az, " Gyro:", gx, gy, gz)

async def main():
    print("Scanning for NanoESP32...")

    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name and "NanoESP32" in d.name
    )

    if not device:
        print("NanoESP32 not found. Make sure it's powered and advertising.")
        return

    print(f"Found device: {device.name} @ {device.address}")

    async with BleakClient(device.address) as client:
        print("Connected!")

        await client.start_notify(CHAR_UUID, notification_handler)

        print("Listening for BLE notifications... (Ctrl+C to stop)")
        while True:
            await asyncio.sleep(1)

asyncio.run(main())
