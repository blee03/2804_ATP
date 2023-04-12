import asyncio, thorpy
from bleak import BleakClient

# address and UUID global variables
ADDRESS = "94:A9:A8:3A:B9:3F"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

async def bob():
    async with BleakClient(ADDRESS) as client:   
        signal = b'1'
        await BleakClient.write_gatt_char(client, UUID, signal, True)
    # Device will disconnect when block exits.

# Using asyncio.run() is important to ensure that device disconnects on
# KeyboardInterrupt or other unhandled exception.

def BLE_startTractor():
    asyncio.run(bob())

application = thorpy.Application(size = (300, 300), caption = "G1 ATP Tractor GUI")
startButton = thorpy.make_button("Start Tractor", func = BLE_startTractor)
stopButton = thorpy.make_button("Stop Tractor", func = BLE_startTractor)

mainMenu = thorpy.Box.make([startButton, stopButton])
mainMenu.center()

background = thorpy.Background(color=(220, 220, 220), elements=[mainMenu])

thorpy.store(background)

menu = thorpy.Menu(elements=background,) #create a menu for auto events handling
menu.play() #launch the menu

application.quit()
