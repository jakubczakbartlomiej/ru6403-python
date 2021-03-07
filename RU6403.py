import serial
import sys
import binascii

class SensorRU6403():
    def __init__(self, address):
        try:
            self.serial = serial.Serial(address, 57600, timeout=2)
            print("Device connected")
            self.getReaderInfo()
        except:
            print(" ** Device not found at " + address + " ** ")
            sys.exit(-1)
            
    def closeConnection(self):
        self.serial.close()
        
    def getReaderInfo(self):
        self.response = self.readerCommunication([0x04, 0x00, 0x21])
        print('Power: ' + str(int(self.response[18:20],16)))
        print('Scan time: ' + str(int(self.response[20:22],16)))
        print('Antena configuration: ' + str(self.response[22:24]))
        print('Antena check: ' + str(int(self.response[28:30],16)))
        print('')
        
    def getWorkingMode(self):
        self.response = self.readerCommunication([0x04, 0x00, 0x77])
        self.readerMode = self.response[6:8]
        self.tagProtocol = self.response[8:10]
        self.filterTime = self.response[12:14]
        self.qValue = self.response[14:16]
        self.session = self.response[16:18]
        self.maskMem = self.response[18:20]
        self.maskAdr = self.response[20:24]
        self.maskLen = self.response[24:26]
        self.maskData = self.response[24:88]
        if(self.readerMode == "00"):
            print("Reader Mode: Answer mode")
        elif(self.readerMode == "01"):
            print("Reader Mode: Real time inventory mode")
        elif(self.readerMode == "02"):
            print("Reader Mode: Real time inventory mode with trigger")
        print("Tag protocol: " + self.tagProtocol)
        print("Filter time: " + self.filterTime)
        print("Q value: " + self.qValue)
        print("Session: " + self.session)
            
    def setScanTime(self, time):
        self.readerCommunication([0x05, 0x00, 0x25, time])
        print("The scan time changed!")
        self.getReaderInfo()
        
    def setRFpower(self, RFpower):
        self.readerCommunication([0x05, 0x00, 0x2F, RFpower])
        #print("The RF Power changed!")
        #self.getReaderInfo()
        
    def setAntennas(self, config):
        '''
            config structure (1 Byte)
            ( bit 7 | bit 6 | bit 5 | bit 4 | bit 3 | bit 2 | bit 1 | bit 0 )
            0 bit: antena 1 config - 1 ON and 0 OFF
            1 bit: antena 2 config - 1 ON and 0 OFF
            2 bit: antena 3 config - 1 ON and 0 OFF
            3 bit: antena 4 config - 1 ON and 0 OFF
            4 bit: always 0
            5 bit: always 0
            6 bit: always 0
            7 bit: preservation status - 1 saving setup when off and 0 discard setup when off
        '''
        self.readerCommunication([0x05, 0x00, 0x3F, config])
        print("The antennas config changed!")
        self.getReaderInfo()                
        
    def inventory(self, qValue, session, target, antenna, scanTime):
        moreFrames = True
        self.fullFrame = ''
        self.epc = []
        self.byteArray = bytearray([0x09, 0x00, 0x01, qValue, session, target, antenna, scanTime])
        self.byteArray.extend(bytearray(self.calculateChecksum(self.byteArray)))
        self.serial.write(self.byteArray)
        while(moreFrames == True):
            self.response = self.serial.read(6)
            self.response = binascii.hexlify(self.response).decode('utf-8')
            self.fullFrame += self.response
            if(self.response[6:8] == "03" and self.response[10:12] == "01"):
                self.response = self.serial.read(int(self.response[0:2],16)-5)
                self.response = binascii.hexlify(self.response).decode('utf-8')
                self.epcLength = int(self.response[0:2],16) * 2 + 2
                self.epc.append(self.response[2:self.epcLength])
                self.fullFrame += self.response
            elif(self.response[6:8] == "01" and self.response[10:12] == "00"): 
                moreFrames = False
                self.serial.read(2)
        #print(self.epc)
        print("Tags amount:" + str(len(self.epc)))
        return self.epc
        
    def readerCommunication(self, cmd):
        self.byteArray = bytearray(cmd)
        self.byteArray.extend(bytearray(self.calculateChecksum(self.byteArray)))
        self.serial.write(self.byteArray)
        self.response = self.serial.read(1)
        self.fullWord = bytearray(self.response)
        self.response = binascii.hexlify(self.response).decode('utf-8')
        self.response = self.serial.read(int(self.response, 16))        
        self.fullWord.extend(bytearray(self.response))
        self.response = binascii.hexlify(self.response).decode('utf-8')
        self.checksum = binascii.hexlify(self.calculateChecksum(self.fullWord[:-2])).decode('utf-8')
        if(str(self.response[4:6]) == "00" and self.checksum == self.response[-4:]):
            return self.response
        else:
            print(" ** Cannot obtain response from the reader ** ")
            sys.exit(-1)
                
    def calculateChecksum(self, array):
        self.uiCrcValue = 0xFFFF
        for ucY in array:
            self.uiCrcValue = self.uiCrcValue ^ ucY
            for ucJ in range(8):
                if self.uiCrcValue & 0x0001:
                    self.uiCrcValue = (self.uiCrcValue >> 1) ^ 0x8408;
                else:
                    self.uiCrcValue = (self.uiCrcValue >> 1)
        self.msb = self.uiCrcValue >> 8
        self.lsb = self.uiCrcValue & 0xFF
        return bytearray([self.lsb, self.msb])
