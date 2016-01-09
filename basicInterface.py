import os
import sys
import math
import threading
import socket
import serial
import time

from threading import Thread

def correctDist(hype, pitch, roll):
    correctPitch = hype * math.cos(pitch)
    correctRoll = correctPitch * math.cos(roll)

    return abs(float(correctRoll))


glob_inc = 0


# changes hexidecimal to float
def interpret(first, second):
    b = hex(first)[2:]
    a = hex(second)[2:]
    if len(a) == 1:
        a = '0'+ a
    if len(b) == 1:
        b = '0'+ b
    full = '0x'+ b + a

    return float(int(full,16))/1000

class serialReader_16:
    def __init__(self, port):
        # opens up serial connection:
        self.baud = 115200
        self.port = port
        self.ser = serial.Serial(self.port,
                                 self.baud,
                                 parity = serial.PARITY_NONE,
                                 stopbits = serial.STOPBITS_TWO,
                                 timeout=0.1)

        self.ser.close()
        self.ser.open()

        self.dist = 0.0

        self.dists = []
        for i in range(99):
            self.dists.append(0.0)

        self.bytes_send = []

        self.CRC = lidarLookup.CRC()

        self.findBlock()

    # calculates checksum
    def calcCRC(self, aBuffer, aLength, aCheck):
        self.lCRCHi = int('0xFF',16)
        self.lCRCLo = int('0xFF',16)

        for i in range(aLength):
            lIndex = self.lCRCLo ^ aBuffer[i]
            self.lCRCLo = self.lCRCHi ^ self.CRC.CRC_HI[lIndex]
            self.lCRCHi = self.CRC.CRC_LO[lIndex]

        if aCheck:
            return([self.lCRCLo, self.lCRCHi])

        aBuffer[aLength] = self.lCRCLo
        aBuffer[aLength + 1] = self.lCRCHi

    def checkCRC(self, raw_item):
        item = [int(str(hex(ord(i))),16) for i in raw_item]
        item_length = len(item)
        check_1 = item[item_length-2]
        check_2 = item[item_length-1]
        new_item = item[:item_length-2]

        #print (check_1,check_2)
        #print (self.calcCRC(new_item, item_length-2, 1))

        if (check_1,check_2) == (self.calcCRC(new_item, item_length-2, 1)):
            print "IT WAS RIGHT! HUZZAH!"
            time.sleep(1.5)
            return True
        else:
            return False

    # sends full serial block
    def sendBlock(self, blockBytes, serPort):
        serPort.write(blockBytes)

    # calculates block to be sent repeatedly
    def findBlock(self):
        #this block appears to return a list of amplitudes:
        #self.bytes_send = [int('0x01',16),int('0x04',16),int('0x00',16),int('0x13',16),int('0x00',16),int('0x0c',16)]

        #this block appears to get distance, according to example code...
        self.bytes_send = [int('0x01',16),int('0x41',16)]

        CRCbytes = self.calcCRC(self.bytes_send, len(self.bytes_send), 1)
        for cur_byte in CRCbytes:
            self.bytes_send.append(cur_byte)

    def listen_thread(self):
        while True:
            while True:
                self.sendBlock(self.bytes_send, self.ser)

                self.data = self.ser.readline()

                print "length: " , len(self.data)

                self.readData = []

                for check in range(299):
                    try:
                        self.readData.append(int(hex(ord(self.data[check])),16))
                        #print check, ": ", self.readData[check]
                    except:
                        break

                #print self.data
                #print ""
                #print ""

                ch = 3 + glob_inc
                for i in range(16): #range(len(self.readData)):
                    if self.data:
                        pass #print self.checkCRC(self.data)

                    try:
                        self.dists[i] = interpret((self.readData[ch]),(self.readData[ch+1]))
                        #self.dists[i] = float(self.readData[ch+1]*256 + self.readData[ch]) / 100.0

                        #if i == 0: self.dists[i] = 10000.0
                        #if i != 8: self.dists[i] = 10000.0

                        #print self.readData[ch], self.readData[ch+1]

                        #print "  ", self.dists[i]

                        #print "DIST1?: ", i, ": ", self.readData[ch+1]*256+self.readData[ch]
                        #print "DIST2?: ", i, ": ", self.readData[ch]*256+self.readData[ch+1]
                        #print "AMP1??: ", i, ": ", (float(self.readData[ch+2])*4+float(self.readData[ch+3]))/64
                        #print "AMP2??: ", i, ": ", (float(self.readData[ch+3])*4+float(self.readData[ch+2]))/64
                        print "SEG??: ", i, ": ", self.readData[ch+4]

                    except:
                        pass
                        #print "NONE?: ", i

                    ch += 5

                print ""
                print ""

                #time.sleep(0.5) #draaag
                break

    # reads and interprets device's output
    def listen(self):
        self.t = threading.Thread(target=self.listen_thread, args=())
        self.t.daemon = True
        self.t.start()
        print "THIS ONE!"

class sensor:
    def __init__(self, port):
        self.threshold = 0
        self.bounceCounter = 0
        self.lastDist = 0

        self.serial = serialReader_16(port)

    # effective debounce filter
    def filter(self, dvar):
        if abs(self.lastDist-dvar)>0.5:
            self.lastDist = dvar
            return self.lastDist
        else:
            self.lastDist = dvar
            return dvar

class copter:
    def __init__(self):
        self.eye = sensor('/dev/ttyUSB0')

        #self.mav = maverick()
        self.dHeight = 0.0
        self.lifted = False
        self.threshold = 0.0
        self.border = 0.0

    def checkAllDist(self):
        self.eye.serial.listen()
        #time.sleep(0.25)

    def checkDist(self, sense):
        if True:
            sense.serial.dist = sense.filter(sense.serial.dist)

        # cancels confirmation if sensor is out of range
        if sense.serial.dist <= 0.0:
            sense.bounceCounter = 0
            sense.serial.dist = sense.lastDist

if __name__ == "__main__":
    print "CONNECTING...",

    coltrane = copter()

    coltrane.border = 2.4

    print "READY!"

    while True:
        #coltrane.checkDist(coltrane.eye)

        sens_data = coltrane.eye.serial.dists

        #if 0.0 in sens_data: print "ZERO READING FROM SENSOR!"

        time.sleep(0.002)

        try:
            pass
        except KeyboardInterrupt:
            break

        #beat.increment += 1
