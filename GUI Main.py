from tkinter import *
import datetime
from signal import signal, SIGTERM, SIGHUP
from rpi_lcd import LCD
import Adafruit_DHT
import smbus
import time
import RPi.GPIO as GPIO
from ctypes import c_short
import serial
import string
import pynmea2
import bmpsensor
# Inputs, Outputs, and More
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
DEVICE = 0x77
bus = smbus.SMBus(1)
sensor = Adafruit_DHT.DHT11
gpio = 24
GPIO.setup(17,GPIO.IN)
lcd = LCD()

# Classes #
class Weather(Frame):
    def __init__(self,master):
        # Main Frame Set Up
        Frame.__init__(self, master, bg="white")
        self.master = master
        # Main Label Setup
        self.l0 = Label(window, text="{}/{}/{} {}:{}:{}".format("00","00","00","00","00","00"), bg='white', font = ("Times_New_Roman",10), bd = 10)
        self.l0.grid(row=0, column=0, rowspan=1, columnspan=7, sticky=N + S + E + W)
        self.l1 = Label(text="{}".format(aa))
        self.l1.grid(row=2, column=0, rowspan=1, columnspan=1, sticky=N + S + E + W)
        self.l2 = Label(text="{}".format(bb))
        self.l2.grid(row=4, column=0, rowspan=1, columnspan=1, sticky=N + S + E + W)
        self.l3 = Label(text="{}".format(cc))
        self.l3.grid(row=2, column=6, rowspan=1, columnspan=1, sticky=N + S + E + W)
        self.l4 = Label(text="{}".format(dd))
        self.l4.grid(row=4, column=6, rowspan=1, columnspan=1, sticky=N + S + E + W)
        self.l5 = Label(text = "Latitude: {} | Longitude: {}".format(lat, log), bg='white', font = ("Times_New_Roman",10), bd = 10)
        self.l5.grid(row = 5, column =0, rowspan = 1, columnspan = 7, sticky = N + S + E + W)
        # Main Functions Setup
        self.GUI()
        self.Update()

    def Click(self,x):
        # Button Click Function
        global aa,bb,cc,dd,ee
        # Buttons
        if x == 1:
            aa = aa + 1
        elif x == 2:
            bb = bb + 1
        elif x == 3:
            cc = cc + 1
        elif x == 4:
            dd = dd + 1
        elif x == 5:
            ee = ee + 1

    def Position(self):
        # GPS Code
        global lat, log
        port="/dev/ttyAMA0"
        ser=serial.Serial(port, baudrate=9600, timeout=1)
        dataout = pynmea2.NMEAStreamReader()
        newdata=ser.readline()
        # Positional Data Recovery
        if newdata[0:6] == b'$GPRMC':
            lat = str(newdata[19:31])
            log = str(newdata[33:45])
        elif newdata[1:7] == b'$GPRMC':
            lat = str(newdata[20:32])
            log = str(newdata[34:46])
    
    def rotate(self):
            #diameter in mm
        cupsdiameter = float(143.8)
            #C = 2pi*r in m
        cupscircumference = float(cupsdiameter/1000)*3.14
            #counts for the inefficiency and error
        errorfactor = float(3.97)
            #sets number of rotations to zero
        rotations = float(0)
            #blackFirst and whiteFirst are set to True to account for when they are detected
        blackFirst = True
        whiteFirst = True
            #endtime = current time + 1
        endtime = time.time() + 1
            #initializes the reading to start
        sensorstart = GPIO.input(17)
        while time.time() < endtime:
                #if the the white is detected
            if GPIO.input(17) == True:
                if blackFirst:
                            #rotation increases by one
                    rotations = rotations + 1
                    blackFirst = False
                    whiteFirst = True
                    print("black")
            else:
                        #if there was a transition from black to white
                if whiteFirst:
                    whiteFirst = False
                    blackFirst = True
                    print("white")
            time.sleep(0.001)
            #if both the rotation and start are True the number of rotations is zero
        if rotations == 1 and sensorstart == 1:
            rotations = 0
            #calculates the rotations per second
        rps = float(rotations/10)
            #calculates the windspeed
        windspeed = float((rps)*cupscircumference*errorfactor)
        return windspeed
    
    def GUI(self):
        global img,num,C,aa,bb,cc,dd,ee
        # Data Time Data
        C = datetime.datetime.now()
        # Button Creation
        self.l10 = Label(self.master, image = img, height = 330, width = 460)
        self.l10.grid(row=1, column=1, rowspan=4, columnspan=3, sticky=N + S + E + W)
        self.b1 = Button(self.master, bg='grey', fg='black', text="  Temperature Label: {}  ".format("C"), borderwidth = 1,
                    highlightthickness=0, activebackground="white", command = lambda:self.Click(1))
        self.b1.grid(row = 1, column = 0,rowspan = 1, columnspan = 1, sticky = N+S+E+W)
        self.b2 = Button(self.master, bg='grey', fg='black', text="  Humidity Label: {}  ".format("%"), borderwidth = 1,
                    highlightthickness=0, activebackground="white", command = lambda:self.Click(2))
        self.b2.grid(row = 3, column = 0,rowspan = 1, columnspan = 1, sticky = N+S+E+W)
        self.b3 = Button(self.master, bg='grey', fg='black', text="  Pressure Label: {}  ".format("Atm"), borderwidth=1,
                    highlightthickness=0, activebackground="white", command = lambda:self.Click(3))
        self.b3.grid(row = 1, column = 6,rowspan = 1, columnspan = 1, sticky = N+S+E+W)
        self.b5 = Button(self.master, bg='grey', fg='black', text="  Speed: {}  ".format("mph"), borderwidth=1,
                    highlightthickness=0, activebackground="white", command = lambda:self.Click(4))
        self.b5.grid(row = 3, column = 6,rowspan = 1, columnspan = 1, sticky = N+S+E+W)
        self.b5 = Button(self.master, bg='grey', fg='black', text="  LDC Line 1: {}  ".format("Loc"), borderwidth=1,
                    highlightthickness=0, activebackground="white", command = lambda:self.Click(5))
        self.b5.grid(row = 1, column = 2,rowspan = 1, columnspan = 1, sticky = N+S+E+W)

    def Update(self):
        # Continuously Updatating Outputs
        global C, i, j, a, lat, log
        # Def Runs
        self.Position()
        S = self.rotate()
        i = i + 1
        C = datetime.datetime.now()
        H, T1 = Adafruit_DHT.read_retry(sensor, gpio)
        T2, Pr, Alt = bmpsensor.readBmp180()
        self.l0.config(text = "{}/{}/{} {}:{}:{}".format(C.month, C.day, C.year, C.hour, C.minute, C.second))
        # Data Orgainization And Type
        if lat != "Loading..." and log != "Loading...":
            self.l5.config(text = "Latitude: {}* {}' | Longitude: {}* {}'".format(lat[2:4],lat[4:13],log[2:4],log[4:13]))
        else:
            lcd.text("Lat: ... | Log: ...",1)
        if (ee % 2) == 1:
            lcd.text("Lat:{}*{}'Log:{}*{}'".format(lat[2:4],lat[4:6],log[2:4],log[4:6]),1)
            self.b5.config(text = "LDC Line 1: {}".format("Time"))
        elif (ee % 2) == 0:
            lcd.text("{}/{}/{} {}:{}:{}".format(C.month, C.day, C.year, C.hour, C.minute, C.second),1)
            self.b5.config(text = "LDC Line 1: {}".format("Loc"))
        Pr = Pr / 1000
        if (aa % 2) == 0:
            T = (T1+T2)/2
            if T == None:
                T = "Error"
            self.b1.config(text="  Temperature Label: {}  ".format("C"))
            self.l1.config(text = "T = {0:0.1f}*C".format(T))
            sym2 = "C"
        elif (aa % 2) == 1:
            T = (T1+T2)/2
            T = (T * 9 / 5) + 32
            if T == None:
                T = "Error"
            self.b1.config(text="  Temperature Label: {}  ".format("F")) 
            self.l1.config(text = "T = {0:0.1f}*F".format(T))
            sym2 = "F"
        self.l2.config(text = "H = {0:0.1f}%".format(H))
        if (cc % 2) == 0:
            Pr = Pr / 101325
            self.b3.config(text="  Pressure Label: {}  ".format("Atm"))
            self.l3.config(text = "P = {0:0.3f}atm".format(Pr))
            sym4 = "Atm"
        elif (cc % 2) == 1:
            self.b3.config(text="  Pressure Label: {}  ".format("Mpa"))
            self.l3.config(text = "P = {0:0.1f}Mpa".format(Pr))
            sym4 = "Mpa"
        S = S * 10
        if (dd % 2) == 0:
            sym4 = "m/s"
            self.l4.config(text = "{:0.4f}m/s".format(S))
        elif (dd % 2) == 1:
            sym4 = "mph"
            S = S * 2.237
            self.l4.config(text = "{:0.4f}mph".format(S))
        # LCD Output
        lcd.text("T = {}*".format(T)+"{}".format(sym2)+" H = {}%".format(H),2)
        lcd.text("S = {:0.4f}".format(S)+"{}".format(sym4),3)
        lcd.text("P = {:.1}".format(Pr)+"{}".format(sym4),4)
        self.after(10, self.Update)

# Definitions #
def convertToString(data):
    return str((data[1] + (256 * data[0])) / 1.2)

def getShort(data, index):
    return c_short((data[index] << 8) + data[index + 1]).value

def getUshort(data, index):
    return (data[index] << 8) + data[index + 1]

def main():
    (temperature,pressure)=readBmp180()
    return temperature, pressure

def safe_exit(signum, frame):
    exit(1)
signal(SIGTERM,safe_exit)
signal(SIGHUP,safe_exit)

def Position():
    # Position outside of Class for loading run
    global lat, log
    port="/dev/ttyAMA0"
    ser=serial.Serial(port, baudrate=9600, timeout=1)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()
    if newdata[0:6] == b'$GPRMC':
        lat = str(newdata[19:31])
        log = str(newdata[33:45])
    elif newdata[1:7] == b'$GPRMC':
        lat = str(newdata[20:32])
        log = str(newdata[34:46])

# Main Code #
# Variable Setup
a = aa = bb = cc = dd = ee = j = 0
num = i = 1
lat = log = "Loading..."
C = datetime.datetime.now()
lcd.text("     Loading...",2)
# First Data Colection
for i in range(1,9):
    inp1 = "-"*i
    inp2 = "_"*(8-i)
    lcd.text("     ({}{})".format(inp1,inp2),3)
    Position()
    time.sleep(0.5)
# Friendly Greeting
for i in range(0,16):
    inp3 = " "*i
    lcd.text("{}Hello".format(inp3),2)
    lcd.text("{}User!".format(inp3),3)
    time.sleep(0.1)
# LCD Reset
lcd.text(" ",2)
lcd.text(" ",3)
# Window Setup
window = Tk()
window.geometry("800x480")
img = PhotoImage(file = "Logo.gif")
W = Weather(window)
window.after(10,Weather.Update)
window.mainloop()