import asyncio
from bleak import BleakClient

# debug flag
debug = True

# address and UUID global variables
ADDRESS = "94:A9:A8:3A:B9:3F"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

def callback(sender: UUID, data: bytearray):
    print(data)
    newData = data.decode('utf-8')
    print(newData)

# main
async def main():
    async with BleakClient(ADDRESS) as client:
        if(debug):
            print(client.is_connected)
            print(client.address)
        
        signal = b'1'

        await BleakClient.write_gatt_char(client, UUID, signal, True)
        print('on')
        await BleakClient.start_notify(client, UUID, callback)   
    # Device will disconnect when block exits.

    ...

# Using asyncio.run() is important to ensure that device disconnects on
# KeyboardInterrupt or other unhandled exception.
asyncio.run(main())
