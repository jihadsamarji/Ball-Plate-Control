from numpy import *
from math import *

def translate(value, leftMin, leftMax, rightMin, rightMax):		# This function is equivalent to the map() function in arduino
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
firstline = "alpha|theta\n"	# define first line used as a title
file.write(firstline) # write first line in text file

maxAlpha = int(14.9*10)	# here is the maximum angle alpha in degrees x10 to scale it
alpha = radians(maxAlpha/10)
#max_theta = arcsin(sin(alpha)*K)
max_theta = 180

for alpha in range(-maxAlpha,maxAlpha+1):
    alpha = radians(alpha/10)
    theta = arcsin(sin(alpha)*K) + pi/2
    alpha = degrees(alpha)
    theta = translate(degrees(theta),20,160,0,180)
    if theta > 120 and theta < 140:						# different areas of non linearities
            theta = translate(theta,120,140,120,145)
    elif theta > 140 and theta < 160:					# different areas of non linearities
            theta = translate(theta,140,160,145,150)
    elif theta > 160 and theta < 180:					# different areas of non linearities
            theta = translate(theta,160,180,150,159)
    #theta = round(degrees(theta),0)
    separator = "|"
    data = str(round(alpha,2)) + separator + str(int(round(theta,0))) + "\n"
    file.write(data)


file.close()

print("Terminé")
