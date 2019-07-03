close all;
clear;
clc;

dm = 0.017; %m    distance from motor shaft to pole
dc = 0.118; %m    distance between center pole and side pole
fc = 0.02 ;%Ns/m  roll friction coefficient
Eta = 1   ;%Damping ratio of the Servo motor
Wn = 7.65 ;%Servo natural Frequency
g = 9.81  ;%m/s^2 gravitational acceleration

% Ball Type    |  Radius |  Mass
% Hollow steel |  0.016  |  0.013
% Massive Iron |  0.01   |  0.046
% Ping Pong    |  0.02   |  0.001
% Marble       |  0.007  |  0.0049

rb = 0.016;%m   radius of the ball
mb = 0.013;%kg  mass of the ball

A = [ 0 1 0 0; 0 (-5/7)*fc/mb (-5/7)*g 0; 0 0 0 1; 0 0 -Wn^2 -2*Eta*Wn]
B = [ 0 ; 0 ; 0 ; Wn^2]
C = [ 1 0 0 0]
D = zeros(1,1)

sys = ss(A,B,C,D)

bode(sys(1,1))

