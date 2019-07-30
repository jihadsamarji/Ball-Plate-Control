clear all
close all
clc
syms x z;
y = pi/8
%syms a b c;
a = 5;
b = 3;
c = 5;

eq1 = sin(z) == (a + b - a*cos(x) - b*cos(y))/c 
eq3 = sqrt((a*cos(x) + b*cos(y) + c*sin(z)^2) + (a*sin(x) + b*sin(y) + c*cos(z))^2) == (a+b)^2 + c^2

%solvez = solve(eq1, z,'ReturnConditions',true)

eq2 = cos(z) == (a + b - a*sin(x) - b*sin(y))/c 

%solvex = solve(eq2,x,'ReturnConditions',true)

eq3 = sqrt((a*cos(x) + b*cos(y) + c*((a + b - a*cos(x) - b*cos(y))/c)^2) + (a*sin(x) + b*sin(y) + c*((a + b - a*sin(x) - b*sin(y))/c))^2) == (a+b)^2 + c^2

solvex = solve(eq3,x,'ReturnConditions',true)

%%
a = 5;
b = 3;
c = 5;
k = 0;
y = pi/8;
x = 2*pi*k - acos(1535930924434819/2814749767106560 - (3*17445^(1/2))/10)
x = x*180/3.1416
