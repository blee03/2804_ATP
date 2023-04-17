import pygame, asyncio
from bleak import BleakClient
from asgiref.sync import async_to_sync

# BLEAK CLIENT DEFINES
ADDRESS = "94:A9:A8:3A:B9:3F"
UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
client = BleakClient(ADDRESS)

# PYGAME DEFINES
SCREEN_HEIGHT = 600
SCREEN_WIDTH = 800
COLOR_GREEN = (119, 186, 153)
COLOR_RED = (211, 63, 73)
COLOR_WHITE = (255, 255, 243)
COLOR_BLACK = (38, 39, 48)

def renderButtons():
    # create start button
    pygame.draw.rect(screen, COLOR_GREEN, [20, 300, 350 , 100], border_radius=20) 
    startText = buttonFont.render('start' , True , COLOR_WHITE)
    screen.blit(startText , (160 , 335))

    # create stop button
    pygame.draw.rect(screen, COLOR_RED, [430, 300, 350 , 100], border_radius=20) 
    stopText = buttonFont.render('emergency stop' , True , COLOR_WHITE)
    screen.blit(stopText , (510 , 335))

    # create trip report button
    pygame.draw.rect(screen, COLOR_WHITE, [340, 420, 120 , 40], border_radius=20) 
    reportText = reportFont.render('trip report' , True , COLOR_BLACK)
    screen.blit(reportText , (365 , 430))

    # create trip report location
    pygame.draw.rect(screen, COLOR_WHITE, [230, 120, 350 , 150], border_radius=20) 
    report_status = statisticFont.render('Status: STOPPED', True , COLOR_RED)
    screen.blit(report_status , (315 , 180))

# CALL BACK FUNCTION FOR RECEIVING DATA
def callback(sender: UUID, data: bytearray):
    decodedData = data.decode('utf-8')
    parsedList = decodedData.split('\t')

    refresh_report()
    report_title = statisticFont.render('Trip Report Statistics', True , COLOR_BLACK)
    screen.blit(report_title , (310 , 160))
    reportTime = statisticFont.render('Time Elapsed: ' + parsedList[0] + 's', True , COLOR_BLACK)
    screen.blit(reportTime , (310 , 180))
    reportTurn = statisticFont.render('Number of Turns:  ' + parsedList[1] , True , COLOR_BLACK)
    screen.blit(reportTurn , (310 , 200))

def refresh_report():
    # create trip report location
    pygame.draw.rect(screen, COLOR_WHITE, [230, 120, 350 , 150], border_radius=20) 

# REFRESH THE TRANSMISSION
@async_to_sync     
async def BLE_start():
    refresh_report()
    report_status = statisticFont.render('Status: DRIVING', True , COLOR_GREEN)
    screen.blit(report_status , (315 , 180))
    try:
        await BleakClient.write_gatt_char(client, UUID, b'1', True)
    except Exception as e:
        print(e)

@async_to_sync     
async def BLE_stop():
    refresh_report()
    report_status = statisticFont.render('Status: STOPPED', True , COLOR_RED)
    screen.blit(report_status , (315 , 180))
    try:
        await BleakClient.write_gatt_char(client, UUID, b'2', True)
    except Exception as e:
        print(e)

# ATTEMPT TO CONNECT TO BLUETOOTH
@async_to_sync
async def BLE_connect():
    try:
        await client.connect()
        print("CONNECTED")
    except Exception as e:
        print(e)

@async_to_sync
async def BLE_recieve():
    try:
        await BleakClient.write_gatt_char(client, UUID, b'3', True)
        await asyncio.sleep(.05) # used to sync between GUI and Arduino
        await BleakClient.start_notify(client, UUID, callback)
    except Exception as e:
        print(e)

# BEGIN MAIN

# CREATE PYGAME SCREEN AND BUTTONS
pygame.init()
screen = pygame.display.set_mode(((SCREEN_WIDTH, SCREEN_HEIGHT)))    
pygame.display.set_caption('Tractor Graphical User Interface')
screen.fill(COLOR_BLACK)
buttonFont = pygame.font.SysFont('none',40) 
reportFont = pygame.font.SysFont('none',22) 
statisticFont = pygame.font.SysFont('none', 30)

# ATTEMPT TO CONNECT BLUETOOTH
renderButtons()
BLE_connect()

# MAIN LOOP
run = True
while run:
    # return position of mouse
    mouse = pygame.mouse.get_pos()
    # check for close event (quit button pressed)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        # check for button presses
        if event.type == pygame.MOUSEBUTTONDOWN:
            if 20 <= mouse[0] <= (20+350) and 300 <= mouse[1] <= 400:
                BLE_start()

        if event.type == pygame.MOUSEBUTTONDOWN:
            if 430 <= mouse[0] <= (430+350) and 300 <= mouse[1] <= 400:
                print('STOP SIGNAL SENT')
                BLE_stop()

        if event.type == pygame.MOUSEBUTTONDOWN:
            if 340 <= mouse[0] <= (340+120) and 420 <= mouse[1] <= 460:
                print('ACCESSING TRIP REPORT')
                BLE_recieve()

    #refresh functions
    pygame.display.update()
pygame.quit()