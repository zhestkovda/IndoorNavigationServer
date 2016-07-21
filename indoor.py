import matplotlib.pyplot as plt
from matplotlib.pyplot import *
import plotly.plotly as py
import numpy as np
import sklearn
from sklearn import preprocessing
import math
import scipy
import sys
import threading
import time
import urllib2
import serial, time
import BaseHTTPServer
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import SimpleHTTPServer
import SocketServer
import json
from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn


PORT_NUMBER = 8080
width  = 2600
height = 4100
xt = 0
yt = 0

def find_position(x1,y1,x2,y2,x3,y3,d11,d22,d33):
    A = np.matrix([[2*x2-2*x1,2*y2-2*y1],[2*x3-2*x2,2*y3-2*y2],[2*x1-2*x3,2*y1-2*y3]])

    row1 = math.pow(d11,2)-math.pow(d22,2)-math.pow(x1,2)+math.pow(x2,2)-math.pow(y1,2)+math.pow(y2,2)
    row2 = math.pow(d22,2)-math.pow(d33,2)-math.pow(x2,2)+math.pow(x3,2)-math.pow(y2,2)+math.pow(y3,2)
    row3 = math.pow(d11,2)-math.pow(d33,2)-math.pow(x1,2)+math.pow(x3,2)-math.pow(y1,2)+math.pow(y3,2)

    f=np.matrix([ row1 ,row2  ,row3  ])
    #print A
    #print f
    coord = []
    try:
        M = np.linalg.inv(A.T*A)
        #print M
        D = M*A.T
        #print D
        coord = D*f.T
    except np.linalg.LinAlgError:
        # Not invertible. Skip this one.
        pass
    return coord

def find_position2(x1,y1,x2,y2,x3,y3,d11,d22,d33):
    A = np.matrix([[2*x2-2*x1,2*y2-2*y1],[2*x3-2*x2,2*y3-2*y2],[2*x1-2*x3,2*y1-2*y3]])

    row1 = math.pow(d11,2)-math.pow(d22,2)-math.pow(x1,2)+math.pow(x2,2)-math.pow(y1,2)+math.pow(y2,2)
    row2 = math.pow(d22,2)-math.pow(d33,2)-math.pow(x2,2)+math.pow(x3,2)-math.pow(y2,2)+math.pow(y3,2)
    row3 = math.pow(d11,2)-math.pow(d33,2)-math.pow(x1,2)+math.pow(x3,2)-math.pow(y1,2)+math.pow(y3,2)

    f=np.matrix([ row1 ,row2  ,row3  ])
    #print A
    #print f
    coord = []
    try:
        M = np.linalg.inv(A.T*A)
        #print M
        D = M*A.T
        #print D
        coord = D*f.T
    except np.linalg.LinAlgError:
        # Not invertible. Skip this one.
        pass
    return coord

def find_position3(x1,y1,x2,y2,x3,y3,d11,d22,d33):
    t1 = math.pow(d11,2) - math.pow(d22,2) - math.pow(x1,2) - math.pow(y1,2) + math.pow(x2,2) + math.pow(y2,2)

    t2 = 2*x2-2*x1
    t3 = 2*y2-2*y1

    t4 = math.pow(d22,2) - math.pow(d33,2) - math.pow(x2,2) - math.pow(y2,2) + math.pow(x3,2) + math.pow(y3,2)

    t5 = 2*x3-2*x2
    t6 = 2*y3-2*y2

    yt = (t2*t4-t1*t5)/(t2*t6-t3*t5+0.00000001)
    xt = (t1-yt*t3)/(t2+0.000000001)
    return xt,yt




def FindTarget():
    global xt
    global yt
    #set the coordinates for A3,A4,A5
    A3 = {'x':0,'y':0}
    A4 = {'x':0,'y':height}
    A5 = {'x':width,'y':0}

    d1=[0] #dist_A3_T1
    d2=[0] #dist_A4_T1
    d3=[0] #dist_A5_T1


    ser = serial.Serial('/dev/tty.usbmodem1411', 115200)
    i=0
    while 1:
        i =  i+1
        if i > 3:

            try:
                line = ser.readline()
                #print line  # If using Python 2.x use: print serial_line
                # Do some other work on the data

                # Loop restarts once the sleep is finished
                #parse the distance
                #ma04 t01 00000387 000002ab 0001 c9 00002449 404a 404a a0
                columns = line.strip().split(' ')
                anchor = columns[0] #ma04 ma05 ma00
                tag = columns[1] #t01
                dist = columns[2] # HEX
                #print anchor[3], int(dist, 16)
            except:
                print "error in COM port connection"
                ser.close()
                ser = serial.Serial('/dev/tty.usbmodem1411', 115200)


            try:
                if tag == "t01":
                    if anchor[3] == "0":
                        temp = int(dist, 16)
                        d1.append(temp)
                    if anchor[3] == "4":
                        temp = int(dist, 16)
                        d2.append(temp)
                    if anchor[3] == "5":
                        temp = int(dist, 16)
                        d3.append(temp)
            except:
                print "error in type conversion"
                ser.close()
                ser = serial.Serial('/dev/tty.usbmodem1411', 115200)

            coord = find_position(int(A3['x']),int(A3['y']),int(A4['x']),int(A4['y']),int(A5['x']),int(A5['y']),d1[-1],d2[-1],d3[-1])
            #print "d1=%f d2=%f d3=%f " % (d1[-1],d2[-1],d3[-1])
            #print coord
            #print "                  \n"
            print coord
            xt = float(coord[0])/float(width)
            yt = float(coord[1])/float(height)
            print "thread #2: %f %f" % (xt,yt)
            time.sleep(1) # sleep 5 sec
    ser.close() # Only executes once the loop exits


# start web server
#This class will handles any incoming request from
#the browser
class myHandler(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type','application/json')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        # Send the html message
        global xt
        global yt
        print "thread #1: %f %f" % (xt,yt)
        #x=0.1 + x
        #y=0.1 + y
        data = {"width": width, "height": height, "tag": { "x":xt, "y":yt} }
        temp = json.dumps(data)
        self.wfile.write(temp)
        return



    def do_HEAD(self):
        self._set_headers()


class ThreadCalcDistance (threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        try:
            FindTarget()
        except KeyboardInterrupt:
            print "Stop calc distance"


def main():
    print "start calc"
    # Create new threads
    thread1 = ThreadCalcDistance(1, "Thread-1", 1) # thread of reading coordinates
    #thread2 = ThreadStartServer(2, "Thread-2", 2)

    # Start new Threads
    thread1.start()
    #thread2.start()

    #Create a web server and define the handler to manage the
    #incoming request
    #thread1.join()
    server = HTTPServer(('', PORT_NUMBER), myHandler)
    print 'Started httpserver on port ' , PORT_NUMBER
    server.serve_forever()

if __name__ == "__main__":
    main()
