import os
import serial
import time
import datetime
import multiprocessing
 
class SerialProcess(multiprocessing.Process):
 
    def __init__(self, taskQ, resultQ):
        multiprocessing.Process.__init__(self)
        self.taskQ = taskQ
        self.resultQ = resultQ
        if os.name == 'nt':
            self.usbPort = 'COM3'
        else:
            self.usbPort = '/dev/ttyACM0'
 
    def close(self):
        self.sp.close()
 
    def sendData(self, data):
        print "sendData start..."
        self.sp.write(data)
        time.sleep(3)
        print "sendData done: " + data
 
    def run(self):

        if 'sp' not in dir(self):
            self.sp = serial.Serial(self.usbPort, 9600, timeout=1)
            print self.sp

        self.sp.flushInput()
 
        while True:
            # look for incoming tornado request
            if not self.taskQ.empty():
                task = self.taskQ.get()
 
                # send it to the arduino
                self.sp.write(str(task) + "\n");
                print "arduino received from tornado: " + task
 
            # look for incoming serial data
            if (self.sp.inWaiting() > 0):
                result = self.sp.readline().strip()

                # epoch = datetime.datetime.now().strftime('%s')
                # data = {'x': int(epoch), 'y': float(result)}
 
                # send it back to tornado
                self.resultQ.put(result)