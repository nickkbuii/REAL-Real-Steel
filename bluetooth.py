import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "180A"
CHAR_UUID = "2A57"

async def notification_handler(sender: int, data: bytes):
    """
    Handle incoming BLE IMU notifications.

    Payload format (CSV, UTF-8):
        ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2

    Units:
        accel: m/s^2
        gyro:  rad/s
    """
    imu_text = data.decode("utf-8").strip()
    parts = imu_text.split(",")
    if len(parts) != 12:
        print("Unexpected payload length:", len(parts), "data:", imu_text)
        return

    (ax1, ay1, az1,
     gx1, gy1, gz1,
     ax2, ay2, az2,
     gx2, gy2, gz2) = map(float, parts)

    print(f"Upper IMU  a[m/s^2]=({ax1:.3f}, {ay1:.3f}, {az1:.3f})"
          f"  w[rad/s]=({gx1:.3f}, {gy1:.3f}, {gz1:.3f})")

    print(f"Forearm IMU a[m/s^2]=({ax2:.3f}, {ay2:.3f}, {az2:.3f})"
          f"  w[rad/s]=({gx2:.3f}, {gy2:.3f}, {gz2:.3f})")


async def main():
    """
    Scan, connect, and subscribe to BLE IMU notifications.

    Parameters
    ----------
    None

    Returns
    -------
    None
    """
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