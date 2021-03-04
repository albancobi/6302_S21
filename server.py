import asyncio
import datetime
import websockets
import serial
import sys

'''Automatically find USB Serial Port
jodalyst 9/2017
edits jkw 8/2019
edits srh 8/2019
'''
import serial.tools.list_ports

PORT = 6305  # needs to be lined up with port specified in file

# Version 2.7 or Above?
if sys.version_info[0] > 2:
    version3 = True
    kwargs = {'newline': ''}
else:
    version3 = False
    kwargs = {}


def port_scan():
    ports = list(serial.tools.list_ports.comports())
    port_dict_all = {i: [ports[i], ports[i].vid] for i in range(len(ports))}
    port_dict = []

    for p in port_dict_all:
        if port_dict_all[p][1] == 5824:
            port_dict.append(port_dict_all[p][0])

    return port_dict

serial_connected = False

async def connect_serial():
    baud = 1000000
    global ser
    global serial_connected
    serial_conn = False
    while True:
        s_list = port_scan()
        for s in s_list:  # Loop through Teensy ports until one responds.
            ser = serial.Serial(port = s[0], 
                                baudrate=baud,
                                parity=serial.PARITY_NONE, 
                                stopbits=serial.STOPBITS_ONE, 
                                bytesize=serial.EIGHTBITS,
                                timeout=0.4) #auto-connects already I guess?

            try:
                data = ser.read(10)
                if len(data) > 8:
                    serial_conn = True
                    break
                else:
                    print("disconnected")
            except Exception as e:
                ser.close()
                serial_conn = False

        if serial_conn:
            break
        
    print("Serial Connected!")
    if ser.isOpen():
        print(ser.name + ' is open...')
        serial_connected = True
        return True

    
async def send_down(message):
    global ser
    global serial_connected
    try:
        ser.write(message.encode('ascii'))
    except Exception as e:
        print("failing on write")
        ser.close()
        serial_connected = False

async def downlink(websocket):
    while True:
        try:
            message = await websocket.recv()
            await send_down(message)
        except:
            break


async def uplink(websocket):
    global ser
    global serial_connected
    while True:
        if not serial_connected:
            await connect_serial()
        try:
            data = ser.read(100) #Larger packets improve efficiency.
            await asyncio.sleep(0.00001) #Needed so page->cpu messages go.
        except Exception as e:
            print("failing on read")
            ser.close()
            serial_connected = False

        try:
            await websocket.send(data)
        except Exception as e:
            break
        
async def handler(websocket, path):
    global serial_connected
    if not serial_connected:
        await connect_serial()
    page2mcu = asyncio.ensure_future(downlink(websocket))
    mcu2page = asyncio.ensure_future(uplink(websocket))
    done, pending = await asyncio.wait([page2mcu,mcu2page],return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()

try:
    server = websockets.serve(handler, "127.0.0.1", PORT)
except:
    print("No server")

try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(server)
    loop.run_forever()
except KeyboardInterrupt:
    print('\nCtrl-C')
finally:
    print("Shutting Down")
    #loop.close()
    #asyncio.get_event_loop().close()

	
