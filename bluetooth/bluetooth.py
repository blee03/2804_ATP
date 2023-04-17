import asyncio
from bleak import BleakClient

# address and UUID global variables
ADDRESS = "94:A9:A8:3A:B9:3F"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# CALL BACK FUNCTION FOR RECEIVING DATA
def callback(sender: UUID, data: bytearray):
    decodedData = data.decode('utf-8') # decode the data into a string
    print (decodedData)

async def main():

    async with BleakClient(ADDRESS) as client:
        signal = b'3'
        await BleakClient.write_gatt_char(client, UUID, signal, True)
        # await asyncio.sleep(0.001)
        await BleakClient.start_notify(client, UUID, callback) 
    # Device will disconnect when block exits.

# Using asyncio.run() is important to ensure that device disconnects on
# KeyboardInterrupt or other unhandled exception.
asyncio.run(main())
