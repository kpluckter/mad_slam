import os
import time
import numpy as np
import scipy as sp
import cv2

class Broadcaster:
    
    def __init__(self):
        self.start_time = time.time()
        self.filename = os.path.join('slam','results.txt')
        self.fileHandle = open(self.filename)
        self.running = True
        self.CamRate = 30 #Hz
        self.GPSRate = 5 #Hz
        self.endTime = 1840.0/30.0
        
    def __del__(self):
        self.fileHandle.close()
        
    def broadcast(self):
        lineThresh = 0
        while True:
            lastThresh = lineThresh
            elapsed_time = time.time() - self.start_time
            #print elapsed_time
            if elapsed_time > self.endTime:
                break
            lineRate = 1.0/self.CamRate
            lineThresh = np.floor(elapsed_time/lineRate).astype(int)   #current line of data
            print lineThresh
            #print lastThresh
            if lineThresh>lastThresh:
                for i, line in enumerate(self.fileHandle):
                    if i == 0:  #skip header lines
                        continue
                    if i == lineThresh:
                        data = line.split(",")
                        self.dx = data[0]
                        self.dy = data[1]
                        self.dz = data[2]
                        self.dh = data[3]
                        print self.dx
                        break;
        
            
    def getUpdate():
        return np.array([self.dx, self.dy,self.dz, self.dh])
    
    
if __name__== "__main__":
        # filename = os.path.join('slam','results.txt')
        # fileHandle = open(filename)
        # for i, line in enumerate(fileHandle):
        #     if i == 0:
        #         continue
        #     data = line.split(",")
        #     dx = data[0]
        #     dy = data[1]
        #     dz = data[2]
        #     print dx
        #     print "\n"
    broad = Broadcaster()
    broad.broadcast()
        #fileHandle.close()