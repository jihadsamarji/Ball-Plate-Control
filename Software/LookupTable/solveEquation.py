from numpy import *
from math import *

def arduino_map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

#values in cm
dm = 1.7 #  distance from motor shaft to pole
dc = 6.2 #   distance between center pole and side pole
K = dc/dm # factor

print("...")

alpha = 0
theta = 0


file = open("data.txt", "w") #create file 
firstline = "alpha|theta\n"
file.write(firstline)

maxAlpha = 149
alpha = radians(maxAlpha/10)
#max_theta = arcsin(sin(alpha)*K)
max_theta = 180

for alpha in range(-maxAlpha,maxAlpha+1):
    alpha = radians(alpha/10)
    if alpha < 0 :
            theta = arcsin(sin(alpha)*K) + pi/2
    else :
            theta = (arcsin(sin(alpha)*K) + pi/2)*0.995
    alpha = round(degrees(alpha),2)
    #theta = translate(degrees(theta),20,160,0,180)
    theta = round(degrees(theta),0)
    separator = "|"
    data = str(alpha) + separator + str(int(theta)) + "\n"
    file.write(data)


file.close()

print("Terminé")
