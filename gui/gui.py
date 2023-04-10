import thorpy, asyncio
from bleak import BleakClient

ADDRESS = "94:A9:A8:3A:B9:3F"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# def callback(sender: UUID, data: bytearray):
#     print(data)
#     newData = data.decode('utf-8')
#     print(newData)

def BLE_startTractor():
    signal = b'1' 
    asyncio.run(sendSignal(signal))

def BLE_stopTractor():
    signal = b'2'
    asyncio.run(sendSignal(signal))

async def sendSignal(signal):
    async with BleakClient(ADDRESS) as client:
        print(client.is_connected)
        print(client.address)
        await BleakClient.write_gatt_char(client, UUID, signal, True)
        print("sent")

async def BLE_tripReport():
    async with BleakClient(ADDRESS) as client:
        await BleakClient.start_notify(client, UUID, callback)   

def callback(sender: UUID, data: bytearray):
    decodedData = data.decode('utf-8') # decode the data into a string
    # since data from arduino is delimited, needs to be split
    parsedList = decodedData.split('\t')
    return parsedList

def callbackTest():
    newData = "temp\ttemp2\ttemp3"
    parsedList = newData.split('\t')
    return parsedList


## BEGIN MAIN ##
application = thorpy.Application(size = (300, 300), caption = "G1 ATP Tractor GUI")

staticTitle = thorpy.make_text("Trip Report", 14, (255,255,255))
staticTime = thorpy.make_text("Elapsed Time", 12, (255,255,255))
staticTapes = thorpy.make_text("# of tapes encountered", 12, (255,255,255))

startButton = thorpy.make_button("Start Tractor", func = BLE_startTractor)
stopButton = thorpy.make_button("Stop Tractor", func = BLE_startTractor)
tripReport = thorpy.make_button("Trip Report", func = BLE_tripReport)

tripReportBox = thorpy.make_ok_box([staticTitle, staticTapes, staticTime])
thorpy.set_launcher(tripReport, tripReportBox)

mainMenu = thorpy.Box.make([startButton, stopButton, tripReport])
mainMenu.center()

background = thorpy.Background(color=(220, 220, 220), elements=[mainMenu])

thorpy.store(background)

menu = thorpy.Menu(elements=background,) #create a menu for auto events handling
menu.play() #launch the menu

application.quit()
