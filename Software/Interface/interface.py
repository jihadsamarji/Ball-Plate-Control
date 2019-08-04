import cv2
import numpy as np
import time
import imutils
import tkinter as tk
import tkinter.messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from math import *

lines = open("data.txt").read().splitlines()  # Read the file data.txt and split it by lines and put it in a list
max_alpha, max_theta = lines[1].split("|")
max_alpha = - float(max_alpha)
max_theta = float(max_theta)

#lines = lines[:-11]  # Remove the last 11 lines ( because they contain descriptions)
#lines = lines[1:]  # Remove the first line ( because it contains the titles )

dataDict = {}

camHeight = 480
camWidth = 640
cam = cv2.VideoCapture(0)  # select which cam device ( 0 is the default ) doesn't give a NULL if no device is connected
cam.set(3, camWidth)    # setting the width of the captured video
cam.set(4, camHeight)   # setting the height of the captured video

getPixelColor = False   # flag to get the pixel color of the ball when needed
H, S, V = 0, 0, 0   # the color properties of the Pixel to track

mouseX, mouseY = 0, 0   # declare variables to capture mouse position for color tracking

for i in range(1, len(lines)):  # for loop to go over the lines list and feed it to the dictionary
    alpha, theta = lines[i].split("|")
    dataDict[float(alpha)] = float(theta)

controllerWindow = tk.Tk()  # initializes this tk interpreter and creates the root window
controllerWindow.title("Control Window")    # define title of the root window
controllerWindow.geometry("820x500")    # define size of the root window
controllerWindow["bg"] = "grey"     # define the background color of the root window
controllerWindow.resizable(0, 0)    # define if the root window is resizable or not for Vertical and horizontal

videoWindow = tk.Toplevel(controllerWindow)     # a new window derived from the root window "controllerwindow"
videoWindow.title("Cam Footage")    # define title of videowindow
videoWindow.resizable(0, 0)  # Cannot resize the window
lmain = tk.Label(videoWindow)   #create an empty label widget in the videoWindow
lmain.pack()    # adjust the size of the videowindow to fit the label lmain
videoWindow.withdraw()  # hide the window

graphWindow = tk.Toplevel(controllerWindow)     # a new window derived from the root window "graphwindow"
graphWindow.title("Position in function of time")   # define window title
graphWindow.resizable(0, 0)     # define if resizable or not
graphCanvas = tk.Canvas(graphWindow, width=camHeight + 210, height=camHeight)       # create a canvas widget in graphwindow
graphCanvas.pack()      # pack the canvas widget
graphWindow.withdraw()  # hide the graphwindow

pointsListCircle = []   # create an empty list to put points refinates that describes a circle patern


def createPointsListCircle(radius):     # create an array of 360 points to describe a whole circle with the argument as radius
    global pointsListCircle
    for angle in range(0, 360):
        angle = angle - 90
        pointsListCircle.append([radius * cos(radians(angle)) + 240, radius * sin(radians(angle)) + 240])


createPointsListCircle(60)     # create a pointsListCircle list that defines a circle of radius 150

pointsListEight = []        # create an empty list to put points refinates that describes an Eight patern


def createPointsListEight(radius):      # create an array of 360 points to describe an Eight shape with the argument as radius
    global pointsListEight
    for angle in range(270, 270 + 360):
        pointsListEight.append([radius * cos(radians(angle)) + 240, radius * sin(radians(angle)) + 240 + radius])
    for angle in range(360, 0, -1):
        angle = angle + 90
        pointsListEight.append([radius * cos(radians(angle)) + 240, radius * sin(radians(angle)) + 240 - radius])


createPointsListEight(40)   # create a pointsListEight list that defines a Eight patern of radius 80

drawCircleBool = False      # flag to draw Circle


def startDrawCircle():      # function triggered by Circle pattern Button as a Toggle
    global drawCircleBool, drawEightBool, refX, refY
    if drawCircleBool == False:
        drawCircleBool = True
        BballDrawCircle["text"] = "Centering the ball"
    else:
        drawCircleBool = False
        refX, refY = 240, 240
        #sliderCoefP.set(sliderCoefPDefault)
        BballDrawCircle["text"] = "moving the ball in circles"


drawEightBool = False


def startDrawEight():       # function triggered by Eight pattern Button as a Toggle
    global drawEightBool, drawCircleBool, refX, refY
    if drawEightBool == False:
        drawEightBool = True
        BballDrawEight["text"] = "Centering the ball"
    else:
        drawEightBool = False
        refX, refY = 240, 240
        #sliderCoefP.set(sliderCoefPDefault)
        BballDrawEight["text"] = "moving the ball in Eights"


pointCounter = 0    # a counter that will cover the whole 360 points in case of draw circle or eight


def drawWithBall():     # function triggered after the startDrawCircle or startDrawEight
    global pointCounter, refX, refY
    if drawCircleBool == True:
        #sliderCoefP.set(15)
        if pointCounter >= len(pointsListCircle):
            pointCounter = 0
        point = pointsListCircle[pointCounter]
        refX, refY = point[0], point[1]
        pointCounter += 7
    if drawEightBool == True:
        #sliderCoefP.set(15)
        if pointCounter >= len(pointsListEight):
            pointCounter = 0
        point = pointsListEight[pointCounter]
        refX, refY = point[0], point[1]
        pointCounter += 7


def setRefWithMouse(mousePosition):   # set refX and refY based on the mousePosition, mousePosition is the realtime position of the mouse not a saved variable
    global refX, refY
    if mousePosition.y > 10:
        refreshGraph()
        refX, refY = mousePosition.x, mousePosition.y
        resetPID()


def getMouseClickPosition(mousePosition):   # get mouse click position
    global mouseX, mouseY
    global getPixelColor
    mouseX, mouseY = mousePosition.x, mousePosition.y
    getPixelColor = True


showVideoWindow = False


def showCameraFrameWindow():    # function to toggle the showVideoWindow and change the label text of the button
    global showVideoWindow, showGraph
    global BShowVideoTxt
    if showVideoWindow == False:
        #if showGraph == True:
        #    graphWindow.withdraw()
        #    showGraph = False
        #    BShowGraph["text"] = "Show Plot"
        videoWindow.deiconify()
        showVideoWindow = True
        BShowVideo["text"] = "Hide Live CAM feed"
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        BShowVideo["text"] = "Show Live CAM feed"


showCalqueCalibrationBool = False       # bool for the toggle of the calibration display on the video window


def showCalqueCalibration():        # function that toggles the showCalqueCalibrationBool
    global showCalqueCalibrationBool
    showCalqueCalibrationBool = not showCalqueCalibrationBool


showGraph = False   # bool for Graph window


def showGraphWindow():  # function that toggles the Graph window and update the show graph button
    global showGraph, showVideoWindow
    global BShowGraph

    if showGraph == False:
        #if showVideoWindow == True:
        #    videoWindow.withdraw()
        #    showVideoWindow = False
        #    BShowVideo["text"] = "Show Live CAM feed"
        showGraph = True
        BShowGraph["text"] = "Hide Plot"
    else:
        showGraph = False
        BShowGraph["text"] = "Show Plot"


t = 480     # time variable for the plotting and initialize at 480 for a good visualization
refY = 240    # reference refinate Y
refX = 240    # reference refinate X


def paintGraph():   # function to plot in realtime the graphWindow
    global t, refY, x, y, prevX, prevY, alpha, prevAlpha
    global showGraphPositionX, showGraphPositionY, showGraphAlpha
    if showGraph == True:
        graphWindow.deiconify()
        if showGraphPositionX.get() == 1:
            graphCanvas.create_line(t - 3, prevX, t, x, fill="#b20000", width=2)
        if showGraphPositionY.get() == 1:
            graphCanvas.create_line(t - 3, prevY, t, y, fill="#0069b5", width=2)
        if showGraphAlpha.get() == 1:
            graphCanvas.create_line(t - 3, 240 - prevAlpha * 3, t, 240 - alpha * 3, fill="#8f0caf", width=2)
        if t >= 480:
            t = 0
            graphCanvas.delete("all")
            graphCanvas.create_line(3, 3, 480, 3, fill="black", width=3)
            graphCanvas.create_line(3, 480, 480, 480, fill="black", width=3)
            graphCanvas.create_line(3, 3, 3, 480, fill="black", width=3)
            graphCanvas.create_line(480, 3, 480, 480, fill="black", width=3)
            graphCanvas.create_line(550, 32, 740, 32, fill="#b20000", width=5)
            graphCanvas.create_line(550, 53, 740, 53, fill="#0069b5", width=5)
            graphCanvas.create_line(550, 73, 740, 73, fill="#8f0caf", width=5)
            if showGraphPositionX.get() == 1:
                graphCanvas.create_line(3, refX, 480, refX, fill="#ff7777", width=2)
            if showGraphPositionY.get() == 1:
                graphCanvas.create_line(3, refY, 480, refY, fill="#6f91f7", width=2)
        t += 3
    else:
        graphWindow.withdraw()


def refreshGraph():     # function that reset the time variable to 480 if the graph is full
    global t
    t = 480


def endProgam():        # function to close root window
    controllerWindow.destroy()


# Defining all the slider default values
sliderHDefault = 15
sliderSDefault = 40
sliderVDefault = 40
sliderCoefPDefault = 0.035
sliderCoefIDefault = 0.0
sliderCoefDDefault = 0.015


def resetSlider():  # function that reset the slider values to default
    sliderH.set(sliderHDefault)
    sliderS.set(sliderSDefault)
    sliderV.set(sliderVDefault)
    sliderCoefP.set(sliderCoefPDefault)
    sliderCoefI.set(sliderCoefIDefault)
    sliderCoefD.set(sliderCoefDDefault)


def donothing():    # function that does nothing, may be used for delay
    pass


def resetPID():    # function that compact the plate
    global totalErrorX, totalErrorY, prevErrorX, prevErrorY, prevIntegX, prevIntegY, prevDerivX, prevDerivY
    totalErrorX = 0
    totalErrorY = 0
    prevErrorX = 0
    prevErrorY = 0
    prevIntegX = 0
    prevIntegY = 0
    prevDerivX = 0
    prevDerivY = 0


def releasePlate():     # function that releases the plate
    global alpha
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Warning", "Are you sure of Releasing the Plate ?"):
            print("Releasing Arms Up")
            ser.write((str(dataDict[0]) + "," + str(dataDict[0]) + "\n").encode())
            alpha = 0
    else:
        if tkinter.messagebox.askokcancel("Warning", "Arduino is not Connected"):
            donothing()


def servosTest():   # function that tests the servos by sweeping
    global max_alpha
    if arduinoIsConnected == True:
        if tkinter.messagebox.askokcancel("Warning", "The plate needs to be in Place"):
            for i in range(1):
                alpha = 0
                beta = 0
                while alpha < max_alpha:
                    ser.write((str(dataDict[alpha]) + "," + str(dataDict[beta]) + "\n").encode())
                    ser.flush()
                    #time.sleep(0.02)
                    alpha = round(alpha + 0.1, 1)
                    print(str(alpha) + "|" + str(dataDict[alpha]))
                while alpha > -max_alpha:
                    ser.write((str(dataDict[alpha]) + "," + str(dataDict[beta]) + "\n").encode())
                    ser.flush()
                    #time.sleep(0.02)
                    alpha = round(alpha - 0.1, 1)
                    print(str(alpha) + "|" + str(dataDict[alpha]))
                while beta < max_alpha:
                    ser.write((str(dataDict[alpha]) + "," + str(dataDict[beta]) + "\n").encode())
                    ser.flush()
                    #time.sleep(0.02)
                    beta = round(beta + 0.1, 1)
                    print(str(beta) + "|" + str(dataDict[beta]))
                while beta > -max_alpha:
                    ser.write((str(dataDict[alpha]) + "," + str(dataDict[beta]) + "\n").encode())
                    ser.flush()
                    #time.sleep(0.02)
                    beta = round(beta - 0.1, 1)
                    print(str(beta) + "|" + str(dataDict[beta]))
                while alpha < max_alpha :
                    ser.write((str(dataDict[alpha]) + "," + str(dataDict[alpha]) + "\n").encode())
                    ser.flush()
                    #time.sleep(0.02)
                    alpha = round(alpha + 0.1, 1)
                    print(str(alpha) + "|" + str(dataDict[alpha]))
                while alpha > -max_alpha :
                    ser.write((str(dataDict[alpha]) + "," + str(dataDict[alpha]) + "\n").encode())
                    ser.flush()
                    #time.sleep(0.02)
                    alpha = round(alpha - 0.1, 1)
                    print(str(alpha) + "|" + str(dataDict[alpha]))
            #time.sleep(5)
            ser.write((str(dataDict[0]) + "," + str(dataDict[0]) + "\n").encode())
    else:
        if tkinter.messagebox.askokcancel("Warning", "Arduino is not Connected"):
            donothing()


arduinoIsConnected = False      # bool to determine if arduino is connected or not


def connectArduino():   # function that checks if arduino is connected or not, initialize the serial and toggles the bools
    global ser
    global label
    global arduinoIsConnected
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" or "CH340" in p.description:
            print(p)
            ser = serial.Serial(p[0], 19200, timeout=1)
            time.sleep(1)  # give the connection a second to settle
            label.configure(text="Arduino connecte", fg="#36db8b")
            arduinoIsConnected = True


startBalanceBall = False    # Bool for the toggling of the Controller


def startBalance():     # function to toggle controller
    global startBalanceBall
    if arduinoIsConnected == True:
        if startBalanceBall == False:
            startBalanceBall = True
            BStartBalance["text"] = "Stop"
        else:
            startBalanceBall = False
            BStartBalance["text"] = "Start"
    else:
        if tkinter.messagebox.askokcancel("Warning", "Arduino is not Connected"):
            donothing()


totalErrorX = 0
totalErrorY = 0
timeInterval = 1
alpha, beta, prevAlpha, prevBeta = 0, 0, 0, 0
omega = 0.1
N = 20  #Derivative Coefficient
prevDerivX = 0 #previous derivative
prevDerivY = 0 #previous derivative
prevIntegX = 0
prevIntegY = 0
delivery_time = 0
prevErrorX = 0
prevErrorY = 0


def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, refX, refY):     # PID controller
    global omega
    global totalErrorX, totalErrorY
    global alpha, beta, prevAlpha, prevBeta
    global startBalanceBall, arduinoIsConnected
    global delivery_time, N
    global prevDerivX, prevDerivY, prevIntegX, prevIntegY
    global prevErrorX, prevErrorY

    errorX = refX - ballPosX
    errorY = refY - ballPosY

    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    Ts = time.time() - delivery_time

    derivX = (prevBallPosX - ballPosX) / Ts
    derivY = (prevBallPosY - ballPosY) / Ts
    Cix = prevIntegX + errorX*Ki*Ts                    #Ki * totalErrorX
    Ciy = prevIntegY + errorY*Ki*Ts                    #Ki * totalErrorX




    Cdx =  (Kd*N*(errorX-prevErrorX)+prevDerivX)/(1+N*Ts)#Ts/(1+N*Ts)*(N*Kd*derivX + prevDerivX/Ts) #Kd * ((errorX - prevErrorX)/Ts)
    Cdy = (Kd*N*(errorY-prevErrorY)+prevDerivY)/(1+N*Ts) #Ts/(1+N*Ts)*(N*Kd*derivY + prevDerivY/Ts) #Kd * ((errorY - prevErrorY)/Ts)

    Ix = Kp * errorX + Cix + Cdx
    Iy = Kp * errorY + Ciy + Cdy

    #Ix = Kp * (refX - ballPosX)
    #Iy = Kp * (refX - ballPosY)

    Ix = round(Ix, 1)
    Iy = round(Iy, 1)



    if Ix > max_alpha:
        Ix = max_alpha
    elif Ix < - max_alpha:
        Ix = - max_alpha
    if Iy > max_alpha:
        Iy = max_alpha
    elif Iy < - max_alpha:
        Iy = - max_alpha

    #alpha = prevAlpha * omega + (1 - omega) * alpha
    #beta = prevBeta * omega + (1 - omega) * beta

    #alpha = round(round(alpha / 0.1) * 0.1, -int(floor(log10(0.1))))  ## permet d'arrondire avec 0.1 de precision
    #beta = round(round(beta / 0.1) * 0.1, -int(floor(log10(0.1))))

    if arduinoIsConnected == True and startBalanceBall == True:
        ser.write((str(dataDict[Ix]) + "," + str(dataDict[-Iy]) + "\n").encode())
        delivery_time = time.time()
    # print(alpha, beta)
    print(Ts)

    if startBalanceBall == True:
        totalErrorX += errorX
        totalErrorY += errorY
        prevDerivX = Cdx
        prevDerivY = Cdy
        prevIntegX = Cix
        prevIntegY = Ciy
        prevErrorX = errorX
        prevErrorY = errorY

prevX, prevY = 0, 0
prevRefX, prevRefY = 0, 0
start_time = 0


def main():     # declaring the main function of the program
    start_timeFPS = time.time()
    global H, S, V
    global getPixelColor
    global x, y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevRefX, prevRefY
    global refX, refY, totalErrorX, totalErrorY
    global camWidth, camHeight
    global timeInterval, start_time
    global showVideoWindow

    _, img = cam.read()     # capturing the image from the cam object, ignore bool, store it in img
    img = img[0:int(camHeight),int((camWidth-camHeight)/2):int(camWidth-((camWidth-camHeight)/2))] #[Y1:Y2,X1:X2]
    #imgCircle = np.zeros(img.shape, dtype=np.uint8)     # create a black image with same size of img
    #cv2.circle(imgCircle, (240, 240), 270, (255, 255, 255), -1, 8, 0)       # create a white mask
    #img = img & imgCircle       # masking
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if getPixelColor == True and mouseY > 0 and mouseY < 480 and mouseX < 480:
        pixelColorOnClick = img[mouseY, mouseX]
        pixelColorOnClick = np.uint8([[pixelColorOnClick]])
        pixelColorOnClick = cv2.cvtColor(pixelColorOnClick, cv2.COLOR_BGR2HSV)
        H = pixelColorOnClick[0, 0, 0]
        S = pixelColorOnClick[0, 0, 1]
        V = pixelColorOnClick[0, 0, 2]
        print(mouseX, mouseY)
        getPixelColor = False

    lowerBound = np.array([H - sliderH.get(), S - sliderS.get(), V - sliderV.get()])
    upperBound = np.array([H + sliderH.get(), S + sliderS.get(), V + sliderV.get()])

    mask = cv2.inRange(imgHSV, lowerBound, upperBound)
    mask = cv2.blur(mask, (6, 6))  # ajoute du flou a l'image
    mask = cv2.erode(mask, None, iterations=2)  # retire les parasites
    mask = cv2.dilate(mask, None, iterations=2)  # retire les parasites

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None

    cv2.circle(img, (int(refX), int(refY)), int(4), (255, 0, 0), 2)
    if showCalqueCalibrationBool == True:
        cv2.circle(img, (240, 240), 220, (255, 0, 0), 2)
        cv2.circle(img, (240, 240), 160, (255, 0, 0), 2)
        cv2.line(img, (240, 240), (240, 240 + 160), (255, 0, 0), 2)
        cv2.line(img, (240, 240), (240 + 138, 240 - 80), (255, 0, 0), 2)
        cv2.line(img, (240, 240), (240 - 138, 240 - 80), (255, 0, 0), 2)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        timeInterval = time.time() - start_time
        (x, y), radius = cv2.minEnclosingCircle(c)
        if radius > 10:
            cv2.putText(img, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            PIDcontrol(int(x), int(y), prevX, prevY, refX, refY)
            start_time = time.time()
    else:
        totalErrorX, totalErrorY = 0, 0

    if showVideoWindow == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    lmain.after(5, main)

    drawWithBall()
    if prevRefX != refX or prevRefY != refY:
        totalErrorX, totalErrorY = 0, 0

    paintGraph()
    prevX, prevY = int(x), int(y)
    prevRefX, prevRefY = refX, refY
    prevAlpha = alpha
    prevBeta = beta

    try:
        print("FPS: ", 1.0 / (time.time() - start_timeFPS))
    except ZeroDivisionError:
        print("FPS: inf")


FrameVideoControl = tk.LabelFrame(controllerWindow, text="Video Control")
FrameVideoControl.place(x=20, y=20, width=380)
EmptyLabel = tk.Label(FrameVideoControl)
EmptyLabel.pack()
BShowVideo = tk.Button(FrameVideoControl, text="Show Live CAM feed", command=showCameraFrameWindow)
BShowVideo.place(x=100, y=-5)
BPositionCalibration = tk.Button(FrameVideoControl, text="Toggle Calibration View", command=showCalqueCalibration)
BPositionCalibration.place(x=230, y=-5)


sliderH = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensibility H", length=350,
                   tickinterval=10)
sliderH.set(sliderHDefault)
sliderH.pack()
sliderS = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensibility S", length=350,
                   tickinterval=10)
sliderS.set(sliderSDefault)
sliderS.pack()
sliderV = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Sensibility V", length=350,
                   tickinterval=10)
sliderV.set(sliderVDefault)
sliderV.pack()

FrameServosControl = tk.LabelFrame(controllerWindow, text="Servos Control")
FrameServosControl.place(x=20, y=315, width=380)
BResetPID = tk.Button(FrameServosControl, text="Reset PID Memory", command=resetPID)
BResetPID.pack()
BElevationBras = tk.Button(FrameServosControl, text="Release Plate", command=releasePlate)
BElevationBras.pack()
BTesterServos = tk.Button(FrameServosControl, text="Test Servomotors", command=servosTest)
BTesterServos.pack()
BStartBalance = tk.Button(FrameServosControl, text="Start", command=startBalance, highlightbackground="#36db8b")
BStartBalance.pack()

FramePIDCoef = tk.LabelFrame(controllerWindow, text="PID coefficients")
FramePIDCoef.place(x=420, y=20, width=380)
BShowGraph = tk.Button(FramePIDCoef, text="Plot on Graph", command=showGraphWindow)
BShowGraph.pack()
sliderCoefP = tk.Scale(FramePIDCoef, from_=0, to=1, orient="horizontal", label="P", length=350, tickinterval=0.001,
                       resolution=0.001)
sliderCoefP.set(sliderCoefPDefault)
sliderCoefP.pack()
sliderCoefI = tk.Scale(FramePIDCoef, from_=0, to=1, orient="horizontal", label="I", length=350, tickinterval=0.001,
                       resolution=0.001)
sliderCoefI.set(sliderCoefIDefault)
sliderCoefI.pack()
sliderCoefD = tk.Scale(FramePIDCoef, from_=0, to=5, orient="horizontal", label="D", length=350, tickinterval=0.001,
                       resolution=0.001)
sliderCoefD.set(sliderCoefDDefault)
sliderCoefD.pack()

FrameBallControl = tk.LabelFrame(controllerWindow, text="Ball Control")
FrameBallControl.place(x=420, y=315, width=380, height=132)
BballDrawCircle = tk.Button(FrameBallControl, text="move the Ball into circle trajectory", command=startDrawCircle)
BballDrawCircle.pack()
BballDrawEight = tk.Button(FrameBallControl, text="move the Ball in Eight trajectory", command=startDrawEight)
BballDrawEight.pack()

label = tk.Label(controllerWindow, text="Arduino disconnected  ", fg="red", anchor="ne")
label.pack(fill="both")
BReset = tk.Button(controllerWindow, text="Reset", command=resetSlider)
BReset.place(x=20, y=460)
BConnect = tk.Button(controllerWindow, text="Connect", command=connectArduino, background="white")
BConnect.place(x=100, y=460)
BQuit = tk.Button(controllerWindow, text="Quit", command=endProgam)
BQuit.place(x=730, y=460)

showGraphPositionX = tk.IntVar()
showGraphPositionX.set(1)
CheckbuttonPositionX = tk.Checkbutton(graphWindow, text="X Position", variable=showGraphPositionX, command=refreshGraph)
CheckbuttonPositionX.place(x=500, y=20)
showGraphPositionY = tk.IntVar()
showGraphPositionY.set(1)
CheckbuttonPositionY = tk.Checkbutton(graphWindow, text="Y Position", variable=showGraphPositionY, command=refreshGraph)
CheckbuttonPositionY.place(x=500, y=40)
showGraphAlpha = tk.IntVar()
CheckbuttonAlpha = tk.Checkbutton(graphWindow, text="Plate Inclination", variable=showGraphAlpha, command=refreshGraph)
CheckbuttonAlpha.place(x=500, y=60)

videoWindow.protocol("WM_DELETE_WINDOW", donothing)
videoWindow.bind("<Button-2>", getMouseClickPosition)
videoWindow.bind("<Button-1>", setRefWithMouse)     # mouse click to set reference position

main()
tk.mainloop()






