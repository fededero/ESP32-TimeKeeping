import os
import eel
import serial
import serial.tools.list_ports
from datetime import datetime

eel.init('web')

global ser
global ports
global portsNumbers
portsNumbers = [0]
ports = serial.tools.list_ports.comports()
ser = serial.Serial()

@eel.expose
def connectToDevice():
    print("")

@eel.expose   
def listPorts():
    portsNumbers.clear()
    for port in ports:
        portsNumbers.append([port.name, port.description])
    eel.listPorts(portsNumbers)

@eel.expose
def connectToCOM(n):
    ser = serial.Serial(ports[int(n)].device, 115200)
    while(True):
        stringRead = ser.readline()
        stringRead = stringRead.decode('utf-8')
        print(stringRead[0:-2])
        eel.characteristicChange(stringRead[0:-2])
        

def closedWindowCallback(pagePath, stillOpen):
    if(len(stillOpen)==0):
        exit(0)



eel.start('TimeKeeping.html', mode='edge', close_callback=closedWindowCallback)
