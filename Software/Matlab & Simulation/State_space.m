close all;
clear;
clc;

dm = 0.017; %m    distance from motor shaft to pole
dc = 0.062; %m    distance between center pole and side pole
fc = 0.001 ;%Ns/m  roll friction coefficient
Eta = 1.0087   ;%Damping ratio of the Servo motor
Wn = 18.915 ;%rad/s Servo natural Frequency
g = 9.81  ;%m/s^2 gravitational acceleration
K = dc/dm;
L = 0.193
% Ball Type    |  Radius |  Mass
% Hollow steel |  0.016  |  0.013
% Massive Iron |  0.01   |  0.046
% Ping Pong    |  0.02   |  0.001
% Marble       |  0.007  |  0.0049

rb = 0.007;%m   radius of the ball
mb = 0.0049;%kg  mass of the ball

%Example of Default parameters
%mb = 0.02
%rb = 0.0125
%Wn = 15
%Eta = 1
%fc = 0.02
%g = 9.81
%dm = 0.03
%dc = 0.05

A = [ 0 1 0 0; 0 (-5/7)*(fc/mb) (+5/7)*g 0; 0 0 0 1; 0 0 -(Wn^2) -2*Eta*Wn]
B = [ 0 ; 0 ; 0 ; Wn^2]
C = [ 1 0 0 0]
D = zeros(1,1)

sys = ss(A,B,C,D)

%R = ctrb(A,B) %controllability matrix of size nxn where n is the
%dimension of the state space

%rank(R) % if full rank, means rank = n then system is controllable

%[U,V,D] = svd(R,'econ') % singular value decomposition of the controlability
%gramian to see the degrees of controllability where one can see how much is 
%each state controllable by the same unit input.

% K = place(A,B,eig) % gives the gain K to place the eigen values of A as
% the variable eig

%bode(sys(1,1))
%% system to visualize the other states

%A2 = [ 0 1 0 0; 0 (-5/7)*(fc/mb) (+5/7)*g 0; 0 0 0 1; 0 0 -(Wn^2) -2*Eta*Wn]
%B2 = [ 0 ; 0 ; 0 ; Wn^2]
%C2 = [ 1 0 0 0; 0 0 1 0; 0 0 K 0; 0 0 0 0]
%D2 = [ 0 ; 0 ; 0; K]

%sys2 = ss(A2,B2,C2,D2)

%% Simscape approximation

% theta = out.Theta.Data;
% alpha = out.Alpha.Data;
% 
% alphaapprox=rad2deg(asin(((dm*sin(deg2rad(theta)))/dc)));
% 
% plot(theta,alpha)
% hold on
% plot(theta,alphaapprox)

