from numpy import *
from math import *

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

maxAlpha = 158
alpha = radians(maxAlpha/10)
max_theta = arcsin(sin(alpha)*K)

for alpha in range(-maxAlpha,maxAlpha+1):
    alpha = radians(alpha/10)
    theta = arcsin(sin(alpha)*K) + pi/2
    separator = "|"
    data = str(round(degrees(alpha),2)) + separator + str(round(degrees(theta),2)) + "\n"
    file.write(data)


file.close()

print("Terminé")
