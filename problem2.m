%%%%%  MECE 7362 Robust Multivariable Control  %%%%%
%%%%%        Final Project - Problem 2         %%%%%
%%%%%           Rahul Vivek Sawant             %%%%%

close all
clear all
clc

% Plant transfer functions
P11 = tf(0.76*[1/0.31 1],conv([1/0.42 1],[1/3.23 1]));
P12 = tf(0.53,conv([1/0.42 1],[1/2.64 1]));
P21 = tf(0.66,conv([1/0.42 1],[1/2.93 1]));
P22 = tf(0.66*[1/0.20 1],conv([1/0.42 1],[1/3.19 1]));
Plant = [P11 P12;P21 P22]

figure(1)
bode(Plant)
grid on
title('Bode plot for diagonally dominant CFTAH system')

% Defining G11
num1 = conv([0.38 1],[5.76 4.8 1])
den1 = conv(conv(conv([1 0],[4.2 1]),[4.2 1]),[0.02 1])
G11 = tf(1.26*num1,den1)
% Defining G22
num2 = conv(conv([2.4 1],[2.4 1]),[0.34 1])
den2 = conv(conv(conv([1 0],[17 1]),[2.4 1]),[0.057 1])
G22 = tf(2.02*num2,den2)
% Overall controller matrix
Controller = [G11 0;0 G22]

%% Decoupling
% Static gain matrix
A = [0.76 0.53; 0.66 0.66]
inv(A)
% New plant
Plant_new = Plant*inv(A)
Plant_new11 = Plant_new(1,1)
Plant_new12 = Plant_new(1,2)
Plant_new21 = Plant_new(2,1)
Plant_new22 = Plant_new(2,2)

% Loop 1 design
L1 = G11*Plant_new11
figure(2);
nichols(L1);
hold on
grid on
% Loop 2 design
L2 = G22*Plant_new22
nichols(L2);
grid on
hold off

%% Problem 2
%
tfinal = 30;
h = 0.01;
time = 0:h:tfinal;
sim problem2_simulink
figure(3)
subplot(4,1,1)
plot(time,Qsys,'linewidth',2)
title('SAWANT: MECE 7362 - Final Project - Problem 2 Simulink Output')
ylim([0 6])
grid on
ylabel('Systemic flow')
subplot(4,1,2)
plot(time,Qpul,'linewidth',2)
ylim([-2 8])
grid on
ylabel('Pulmonary flow')
subplot(4,1,3)
plot(time,VL,'linewidth',2)
ylim([0 6])
grid on
ylabel('Left Pump Voltage')
subplot(4,1,4)
plot(time,VR,'linewidth',2)
ylim([-2 6])
grid on
ylabel('Right Pump Voltage')
xlabel('Time')
%