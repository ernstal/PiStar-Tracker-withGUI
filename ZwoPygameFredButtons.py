###########################################################
# capturing video via threading, convert it, make surface
# and represent it via pygame
# pip3 install zwoasi
# pip install zwoasi (thanks to Steve Marple for the python wrapper)
# Download asi-sdk and copy the armv7 dynamic lib to /lib/zwoasi/armv7/ 


import cv2
import numpy as np
import os
import threading
import pygame
from pygame.locals import *
import sys
import zwoasi as asi
import time
from PIL import Image
from time import gmtime, strftime

########################## Imported from old program ###############################################
TrackedStar = 0

CalibrationSteps = 100
StartCalibration = 0
RACalibIndicator = 0
RALog = np.empty((0,2),float)

DECCalibIndicator = 0
DECLog = np.empty((0,2),float)

slopeRA = 0
slopeDEC = 0
slopeIndicator = 0

StarTableTrans = np.empty((0,4),float)

dx = 0
dy = 0

TrackingIndicator = 0

ShowLog = 1

RAangle = 0
DECangle = 0

def getCoordinates():
    frame = tframe
    TempCoordinates_ = np.empty((0,2),float)
    Threshold_ = 120
    ret, thresh_ = cv2.threshold(tframe,Threshold_,255,0)
    image, cnts, hierarchy = cv2.findContours(thresh_, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        M = cv2.moments(c)
        if(M['m10']!=0):
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            #print("("+str(cX)+"/"+str(cY))
            TempCoordinates_ = np.append(TempCoordinates_,np.array([[cX,cY]]),axis=0)
    return TempCoordinates_

def getInitialStarTable():
    Coordinates = getCoordinates()
    TempStarTable = np.empty((0,4),float)
    StarNumber = 0
    print(Coordinates)
    for x,y in Coordinates:
        TempStarTable = np.append(TempStarTable,np.array([[StarNumber,x,y,1]]),axis=0)
        StarNumber+=1
    return TempStarTable

def StarMatching(StarTable):
    CorrMatrix = np.empty((0,2),int)
    
    CurrentCoordinates = getCoordinates()
    i,x0,y0,onlineTable = StarTable.T
    xN,yN = CurrentCoordinates.T
    onesArrA = np.ones((StarTable.shape[0],1))
    onesArrB = np.ones((CurrentCoordinates.shape[0],1))
    x0 = np.outer(x0,onesArrB)
    xN = np.outer(onesArrA,xN)
    y0 = np.outer(y0,onesArrB)
    yN = np.outer(onesArrA,yN)
    dx = np.power(x0-xN,2)
    dy = np.power(y0-yN,2)
    R = np.sqrt(dx+dy)
    sigma = 20
    G=np.exp(-(R*R)/(2*sigma*sigma))
    G=np.round(G,0)

    i=0
    j=0
    
    while(i<G.shape[0]):
        while(j<G.shape[1]):
            if(G[i,j]==1):
               CorrMatrix = np.append(CorrMatrix,np.array([[i,j]]),axis=0)
            j+=1
        j=0
        i+=1
    for C in CorrMatrix:
        c1,c2 = C
        starnumber,x,y,online = StarTable[c1]
        xnew,ynew = CurrentCoordinates[c2]
        StarTable[c1]=starnumber,xnew,ynew,1
    return StarTable

def getSlope():
    global slopeRA
    global slopeDEC
    global RAangle
    global DECangle
    
    xRA, yRA = RALog.T
    xDEC, yDEC = DECLog.T

    slopeRA, zRA = np.polyfit(xRA,yRA,1)
    slopeDEC, zDEC = np.polyfit(xDEC,yDEC,1)
    RAangle = np.degrees(np.arctan(slopeRA))
    DECangle = np.degrees(np.arctan(slopeDEC))

def RACalibration(StarTable,TrackedStar):
    
    global RACalibIndicator
    global RALog
    if(len(RALog) <= (CalibrationSteps-1)):
        GPIO.output(20,1)
        RALog = np.append(RALog,np.array([[StarTable[TrackedStar,1],StarTable[TrackedStar,2]]]),axis=0)
    if(len(RALog) == CalibrationSteps):
        RACalibIndicator = 1
        GPIO.output(20,0)
    
def DECCalibration(StarTable,TrackedStar):
    global DECCalibIndicator
    global DECLog
    if(len(DECLog) <= (CalibrationSteps-1) and RACalibIndicator == 1):
        GPIO.output(19,1)
        DECLog = np.append(DECLog,np.array([[StarTable[TrackedStar,1],StarTable[TrackedStar,2]]]),axis=0)
    if(len(DECLog) == CalibrationSteps):
        GPIO.output(19,0)
        DECCalibIndicator = 1

def CoordinatesTransformation(StarTable,TrackedStar):
    global StarTableTrans
    global xreference
    global yreference

    StarTableTrans = np.empty((0,4),float)
    for Star in StarTable:
        number, x, y, online = Star
        x = x-xreference
        y = y-yreference
        motionAnglex = abs(DECangle)
        motionAngley = abs(RAangle)
        xtrans = x*np.cos(motionAnglex * np.pi/180)-y*np.sin(motionAnglex * np.pi/180)
        ytrans = x*np.sin(motionAnglex * np.pi/180)+y*np.cos(motionAnglex * np.pi/180)
        #xtrans = x*np.cos(slopeDEC)+y*np.sin(slopeDEC)
        #ytrans = y*np.cos(slopeDEC)-x*np.sin(slopeDEC)
        StarTableTrans = np.append(StarTableTrans,np.array([[number,xtrans,ytrans,1]]),axis=0)

################ Init everything ###################################################################
asi.init("/lib/zwoasi/armv7/libASICamera2.so")
camera = asi.Camera(0)
global ExpTime
global CamGain
global tframe
global tresh
global TempCoordinates
global StarTable
global SearchIndikator
global TrackIndikator
global selectedStarNumber
global selectedStar
global selectedStarIndikator
global calibIndi
TempCoordinates = np.empty((0,2),float)
ExpTime = 100
CamGain = 50
tframe = np.zeros((480,640,3))
thresh = np.zeros((480,640,1))
pygame.init()
###################################  GPIOs ######################################################
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(26,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)

GPIO.output(26,0) #RA#
GPIO.output(19,0) #DEC-
GPIO.output(20,0) #RA+
GPIO.output(21,0) #DEC+

def closeGPIO():
    GPIO.output(26,0) #RA-
    GPIO.output(19,0) #DEC-
    GPIO.output(20,0) #RA+
    GPIO.output(21,0) #DEC+
    GPIO.cleanup()

################# camera initial settings #########################################################
camera.set_control_value(asi.ASI_GAIN, CamGain)
camera.set_control_value(asi.ASI_EXPOSURE, 100)
camera.set_control_value(asi.ASI_WB_B, 99)
camera.set_control_value(asi.ASI_WB_R, 75)
camera.set_control_value(asi.ASI_GAMMA, 50)
camera.set_control_value(asi.ASI_BRIGHTNESS, 150)
camera.set_control_value(asi.ASI_FLIP, 0)
camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, camera.get_controls()['BandWidth']['MinValue'])
camera.set_roi_format(640,480,1,0)
camera.auto_wb()
camera.start_video_capture()
####################################################################################################
pygame.display.set_caption("Freded window")
screen = pygame.display.set_mode([800,600])
pygame.font.init()
myfont2 = pygame.font.SysFont('Arial',15)
##################### Buttons ######################################################################

button_calib = pygame.image.load("buttons/button_calib.png")
button_capimg = pygame.image.load("buttons/button_capim.png")
button_expminus = pygame.image.load("buttons/button_emin.png")
button_expplus = pygame.image.load("buttons/button_eplus.png")
button_exit = pygame.image.load("buttons/button_exit.png")
button_gminus = pygame.image.load("buttons/button_gmin.png")
button_gplus = pygame.image.load("buttons/button_gplus.png")
button_search = pygame.image.load("buttons/button_search.png")
button_select = pygame.image.load("buttons/button_select.png")
button_track = pygame.image.load("buttons/button_track.png")
button_select_star = pygame.image.load("buttons/button_select-star.png")
button_aim_star = pygame.image.load("buttons/button_aim-star.png")
button_calibration = pygame.image.load("buttons/button_calib.png")
################## Display Text and Buttons Func ###################################################
def ShowText():
    screen.blit(SensorTempText,(20,40))
    screen.blit(StarsFoundText,(20,60))
    screen.blit(ExpTimeText,(20,80))
    screen.blit(CamGainText,(20,100)) 

    
    screen.blit(button_exit,(651,5))
    screen.blit(button_select,(650,50))
    screen.blit(button_track,(650,95))
    screen.blit(button_capimg,(650,140))
    screen.blit(button_expplus,(650,185))
    screen.blit(button_expminus,(650,230))
    screen.blit(button_gplus,(650,275))
    screen.blit(button_gminus,(650,320))
    screen.blit(button_select_star,(650,365))
    screen.blit(button_aim_star,(650,410))
    screen.blit(button_calibration,(5,485))
################ Check Mouse Position for Button ##################################################
def CheckMousePos():

    global SearchIndikator
    xmouse,ymouse = pygame.mouse.get_pos()
    ButtonSleeptime = 0.2

    if(ymouse < 50):
        ExitButton()
        time.sleep(ButtonSleeptime)

    if(ymouse > 51 and ymouse < 91 and xmouse > 650 and xmouse < 800):
        SelectButton()

        
    if(ymouse > 96 and ymouse < 135 and xmouse > 650 and xmouse < 800):
        TrackButton()
        time.sleep(ButtonSleeptime)
     
    if(ymouse >141 and ymouse < 181 and xmouse > 650 and xmouse < 800):
        CapImgButton()
        time.sleep(ButtonSleeptime)
        
    if(ymouse >185 and ymouse < 225 and xmouse > 650 and xmouse < 800):
        ExpPlusButton()
        #time.sleep(ButtonSleeptime)
        
    if(ymouse >231 and ymouse < 269 and xmouse > 650 and xmouse < 800):
        ExpMinusButton()
        #time.sleep(ButtonSleeptime)

    if(ymouse >277 and ymouse < 313 and xmouse > 650 and xmouse < 800):
        GainPlusButton()
        time.sleep(ButtonSleeptime-0.1)

    if(ymouse >320 and ymouse < 360 and xmouse > 650 and xmouse < 800):
        GainMinusButton()
        time.sleep(ButtonSleeptime-0.1)
        
    if(ymouse >360 and ymouse < 405 and xmouse > 650 and xmouse < 800):
        selectStar()
        time.sleep(ButtonSleeptime-0.1)
        
    if(ymouse >403 and ymouse < 450 and xmouse > 650 and xmouse < 800):
        aimStar()
        time.sleep(ButtonSleeptime)
        
    if(ymouse >485 and ymouse < 529 and xmouse > 5 and xmouse < 160):
        calibrationIndikator()
        time.sleep(ButtonSleeptime)
        
######################### Button Control Functions ############################################
def showCalibText():
    global calibIndi
    if (calibIndi == 1 and DECCalibIndicator == 0 and RACalibIndicator == 0):
        CalibText =  myfont2.render("RA Calibration on",False,(255,255,255))
        screen.blit(CalibText,(5,460))
    if (calibIndi == 1 and DECCalibIndicator == 0 and RACalibIndicator == 1):
        CalibText =  myfont2.render("DEC Calibration on",False,(255,255,255))
        screen.blit(CalibText,(5,460))
    if (DECCalibIndicator == 1 and RACalibIndicator == 1):
        CalibSucceedText =  myfont2.render("Calibration succeeded",False,(255,255,255))
        screen.blit(CalibSucceedText,(5,460))

def calibrationIndikator():
    global calibIndi
    global DECCalibIndicator
    global RACalibIndicator
    global DECLog
    global RALog
    
    calibIndi = (calibIndi + 1)%2

    if(DECCalibIndicator == 1 and RACalibIndicator == 1):
        DECCalibIndicator = 0
        RACalibIndicator = 0
        RALog = np.empty((0,2),float)
        DECLog = np.empty((0,2),float)
        getSlope()
        
def calibration():
    global StarTable
    global selectedStarNumber
    global RACalibIndicator
    
    global RA
    if(calibIndi == 1):
        RACalibration(StarTable,selectedStarNumber)
    if(RACalibIndicator == 1):
        DECCalibration(StarTable,selectedStarNumber)
def aimStar():
    global selectedStarIndikator
    selectedStarIndikator = (selectedStarIndikator+1)%2

def drawSelectedStar():
    global selectedStarIndikator
    global selectedStar
    if(selectedStarIndikator == 1):
        number,x,y,online = selectedStar
        pygame.draw.line(screen,(255,255,255),(0,0),(x,y))

        size = 20
        x11 = x+size
        x12 = x+size
        x13 = x+size/2
        y11 = y-size
        y12 = y-size/2
        y13 = y-size

        pygame.draw.line(screen,(255,255,255),(int(x11),int(y11)),(int(x12),int(y12)))
        pygame.draw.line(screen,(255,255,255),(int(x11),int(y11)),(int(x13),int(y13)))

        x21 = x+size
        x22 = x+size
        x23 = x+size/2
        y21 = y+size
        y22 = y+size/2
        y23 = y+size

        pygame.draw.line(screen,(255,255,255),(int(x21),int(y21)),(int(x22),int(y22)))
        pygame.draw.line(screen,(255,255,255),(int(x21),int(y21)),(int(x23),int(y23)))

        x31 = x-size
        x32 = x-size
        x33 = x-size/2
        y31 = y+size
        y32 = y+size/2
        y33 = y+size

        pygame.draw.line(screen,(255,255,255),(int(x31),int(y31)),(int(x32),int(y32)))
        pygame.draw.line(screen,(255,255,255),(int(x31),int(y31)),(int(x33),int(y33)))

        x41 = x-size
        x42 = x-size
        x43 = x-size/2
        y41 = y-size
        y42 = y-size/2
        y43 = y-size

        pygame.draw.line(screen,(255,255,255),(int(x41),int(y41)),(int(x42),int(y42)))
        pygame.draw.line(screen,(255,255,255),(int(x41),int(y41)),(int(x43),int(y43)))

    
def selectStar():
    global StarTable
    global selectedStar
    global selectedStarNumber
    global selectedStarIndikator

    selectedStarNumber = selectedStarNumber +1
    selectedStarIndikator = (selectedStarIndikator +1)%2
    if(selectedStarNumber == len(StarTable)):
       selectedStarNumber = 0
    selectedStar = StarTable[selectedStarNumber]
    print(selectedStar)


    
def ExitButton():
    print('Exit')
    print(pygame.mouse.get_pos())
    pygame.quit();
    closeGPIO()

def SelectButton():
    Search()
    global StarTable
    print("Select")
    StarTable = getInitialStarTable()
    print(StarTable)

def TrackButton():
    print("Track")
    global TrackIndikator
    TrackIndikator = (TrackIndikator +1)%2
    if(TrackIndikator == 0):
        print("Tracking Off")
    if(TrackIndikator == 1):
        print("Tracking On")
    print(StarTable)

def CapImgButton():
    print('CapImg')
    im = Image.fromarray(tframe)
    im.save(strftime("%Y-%m-%d-%H:%M:%S",gmtime())+"img.jpeg")
    print("CapImg")

def ExpPlusButton():
    global ExpTime
    ExpTime = int(ExpTime * 1.3)
    print("E+")
    
def ExpMinusButton():
    global ExpTime
    ExpTime = int(ExpTime * 0.75)
    print("E-")
    
def GainPlusButton():
    global CamGain
    if(CamGain == 150):
        CamGain = 150
    else:
        CamGain+=10
    print("G+")
    
def GainMinusButton():
    global CamGain
    if(CamGain == 10):
        CamGain = 10
    else:
        CamGain-=10
    print("G-")    

def drawKS():
        global StarTable
        global calibIndi
        global DECCalibIndicator
        global RACalibIndicator
        global RAangle
        global DECangle
        global selectedStar
        
        if (calibIndi == 1 and DECCalibIndicator == 1 and RACalibIndicator == 1):
            length = 100
            xreference = selectedStar[1]
            yreference = selectedStar[2]
            x2 = xreference + length * np.cos(RAangle * np.pi/180)
            y2 = yreference + length * np.sin(RAangle * np.pi/180)
            x3 = xreference + length * np.cos((DECangle) * np.pi/180)
            y3 = yreference + length * np.sin((DECangle) * np.pi/180)

            pygame.draw.line(screen,(255,0,0),(int(xreference),int(yreference)),(int(x2),int(y2)))
            pygame.draw.line(screen,(0,0,255),(int(xreference),int(yreference)),(int(x3),int(y3)))
            pygame.display.flip()

            RAKS =  myfont2.render("RA+",False,(255,0,0))
            DECKS =  myfont2.render("DEC+",False,(255,0,0))
 
            screen.blit(RAKS,(int(x2),int(y2)))
            screen.blit(DECKS,(int(x3),int(y3)))

            pygame.display.update()
            

################### Init Controls ##############################################################
SearchIndikator = 0
TrackIndikator = 0
selectedStarNumber = 0
selectedStarIndikator = 0
calibIndi = 0
################################################################################################
def Search():
    global thresh
    global TempCoordinates
    global SearchIndikator
    SearchIndikator=(SearchIndikator+1)%2
    
    if(SearchIndikator == 1):
        TempCoordinates = np.empty((0,2),float)
        Threshold = 120
        #blurred = cv2.blur(tframe,(2,2))
        #blurred = cv2.bilateralFilter(tframe,11,17,17)
        reth,thresh = cv2.threshold(tframe,Threshold,255,0)
        image, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            M = cv2.moments(c)
            if(M['m10']!=0):
                cX = int(M['m10']/M['m00'])
                cY = int(M['m01']/M['m00'])
                TempCoordinates = np.append(TempCoordinates,np.array([[cX,cY]]),axis=0)  
############################# Tracking #########################################################
def Tracking():
    global TrackIndikator
    global StarTable
    if(TrackIndikator == 1):
        Coordinates = getCoordinates()
        StarTable = StarMatching(StarTable)
        drawStars()
################ Draw Stars ####################################################################
def drawStars():
    global StarTable
    if(len(StarTable) > 0):
        for starnumber,x,y,onlineIndi in StarTable:
            pygame.draw.circle(screen,(255,0,0),(int(x),int(y)),3)
            StarText =  myfont2.render(str(starnumber)+"("+str(x)+"/"+str(y)+")",False,(255,255,255))
            screen.blit(StarText,(x,y)) 
    
################## Capture Fred ################################################################
class getFrame(threading.Thread):
    def __init__ (self):
        threading.Thread.__init__(self)
        self.stopThread = False

    def run(self):
        while True:
            global tframe
            global ExpTime
            camera.set_control_value(asi.ASI_EXPOSURE, ExpTime)
            camera.set_control_value(asi.ASI_GAIN, CamGain)
            tframe = camera.capture_video_frame()
            if self.stopThread == True:
                break
    def stopThread(self, stopThread):
        self.stopThread = stopThread
######### Convert frame to gray for surface######################################################
######### pygame want's it this way #############################################################
def gray (im):
    im = 255* (im/im.max())
    w,h=im.shape
    ret = np.empty((w,h,3),dtype=np.uint8)
    ret[:,:,2] = ret[:,:,1] = ret[:,:,0] = im
    return ret
#################### Start Fred #################################################################
t1 = getFrame()
t1.start()
#################################################################################################

##############warm up camera before start########################################################
time.sleep(1)
i=0
while(i==5):
    tframe = camera.capture()
    i+=1
#################################################################################################


try:
    while True:
        SensorTemp = (asi.ASI_TEMPERATURE * 10 - 32)/1.8 #convert fahrenheit to real unit
        SensorTemp = int(SensorTemp)
        SensorTempText =  myfont2.render("Sensor Temp: " + str(SensorTemp) + " °C",False,(255,0,0))
        ExpTimems = ExpTime / 1000
        ExpTimeText =  myfont2.render("Exp Time: " + str(ExpTimems) + " ms",False,(255,0,0))
        StarsFoundText =  myfont2.render("Stars: " + str(len(TempCoordinates)),False,(255,0,0))
        CamGainText =  myfont2.render("Gain: " + str(CamGain),False,(255,0,0))
        screen.fill([0,0,0])
        
        if(SearchIndikator == 0):
            frame = gray(tframe)
        if(SearchIndikator == 1):
            Search()
            frame = gray(thresh)

        frame = np.rot90(frame)
        frame = np.rot90(frame)
        frame = np.rot90(frame)
        frame = np.fliplr(frame)
        frame = pygame.surfarray.make_surface(frame)
        screen.blit(frame,(0,0))


        #print(pygame.mouse.get_pos())

        #Always draw button before text ->overlay
        #DrawButtons()
        ShowText()
        if(SearchIndikator == 1):
            drawStars()
            
        Tracking()
        drawSelectedStar()
        calibration()
        showCalibText()
        pygame.display.update()

        ############### Draw KS ##################
        drawKS()
        
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                sys.exit(0)
        if event.type == pygame.MOUSEBUTTONDOWN:
            CheckMousePos()

except (KeyboardInterrupt,SystemExit):
    pygame.quit()
    cv2.destroyAllWindows()
