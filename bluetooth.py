import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "180A"
CHAR_UUID = "2A57"

async def notification_handler(sender: int, data: bytes):
    """
    Handle incoming BLE IMU notifications.

    Parameters
    ----------
    sender : int
        BLE characteristic handle that sent the notification.
    data : bytes
        Raw BLE payload in the format:
        "uq0,uq1,uq2,uq3,fq0,fq1,fq2,fq3"
        where:
            uq* = shoulder IMU quaternion components (w, x, y, z)
            fq* = forearm IMU quaternion components (w, x, y, z)

    Returns
    -------
    None
    """
    imu_text = data.decode("utf-8").strip()
    print(f"Notification from {sender}: {imu_text}")

    parts = imu_text.split(",")
    if len(parts) != 8:
        print("Warning: Received malformed quaternion packet:", imu_text)
        return

    uq0, uq1, uq2, uq3, fq0, fq1, fq2, fq3 = map(float, parts)

    shoulder_q = (uq0, uq1, uq2, uq3)
    forearm_q  = (fq0, fq1, fq2, fq3)

    print("Shoulder quat:", shoulder_q)
    print("Forearm  quat:", forearm_q)

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