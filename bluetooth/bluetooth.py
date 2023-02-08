import asyncio
from bleak import BleakClient

debug = True


ADDRESS = "94:A9:A8:3A:B9:3F"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

async def main():
    async with BleakClient(ADDRESS) as client:
        if(debug):
            print(client.is_connected)
            print(client.address)
        ...
        signal = b'1'
        # await BleakClient.write_gatt_char(client, UUID, signal, True)

        output = await BleakClient.read_gatt_char(client, UUID)
        print(output)

    # Device will disconnect when block exits.
    ...

# Using asyncio.run() is important to ensure that device disconnects on
# KeyboardInterrupt or other unhandled exception.
asyncio.run(main())