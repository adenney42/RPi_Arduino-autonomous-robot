#!/usr/bin/env python
import serial
import time


try:
    port = serial.Serial(
    port='/dev/ttyACM0',
    baudrate = 57600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )
    time.sleep(2)
except:
    print("serial open error")
    
run = True
battVoltage = 0
lightLevel = 0
ypr = [0,0,0]
acc = [0,0,0]
utRange = 0
print("Batt: {} VDC  Light: {}  Range: {} cm  X: {}  Y: {}  Z: {}".format(battVoltage, lightLevel, utRange, ypr[0], ypr[1], ypr[2])) 


def sendCommand(serCommand):    # command range 4000 to 8000
    port.flush()
    serCommand_encode = serCommand + "\n"
    port.write(serCommand_encode)
    print("Sent command:  ", serCommand)
    time.sleep(0.5)
    
def readSerial():
    port.flush()
    fromArduino = port.readline()
    return fromArduino
    
    
def moveAround():
    sendCommand("mf")

def main():
    battVoltage = 0
    lightLevel = 0
    ypr = [0,0,0]
    acc = [0,0,0]
    utRange = 0
    
    while (run):
        sensorData = readSerial()
        #print('Input: ', sensorData)
        if (sensorData[0] == 'v'):      # voltage sensor data vX.XX
            battVoltage = float(sensorData[1:-2])
        if (sensorData[0] == 'p'):      # light level data pXXX
            lightLevel = float(sensorData[1:-2])
        if (sensorData[0] == 'd'):      # ultrasonic distance data dXXX   
            utRange = int(sensorData[1:-2])
        if (sensorData[0] == 'y'):      # gyro data yprX:Y:Z
            yprTemp = sensorData[3:-2]
            i = 0
            yprList = []
            for n in yprTemp:
                if n != ':':
                    yprList.append(n)
                else:
                    ypr[i] = ''.join(yprList)
                    yprList = []
                    i += 1
        if (sensorData[0] == 'a'):      # acceleration data accX:Y:Z
            accTemp = sensorData[3:-2]
            i = 0
            accList = []
            for n in accTemp:
                if n != ':':
                    accList.append(n)
                else:
                    acc[i] = ''.join(accList)
                    accList = []
                    i += 1
                    
        #time.sleep(5)
        print("Batt: {} VDC  Light: {}  Range: {} cm  X: {}  Y: {}  Z: {}".format(battVoltage, lightLevel, utRange, ypr[0], ypr[1], ypr[2])) 
    time.sleep(1)
    port.close()

if __name__ == '__main__':
    main()
