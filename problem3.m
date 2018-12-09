%%%%%  MECE 7362 Robust Multivariable Control  %%%%%
%%%%%        Final Project - Problem 3         %%%%%
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
num1 = conv(conv(conv([5 1],[2.7 1]),[4.84 4.4 1]),[0.1296 0.72 1])
den1 = conv(conv(conv(conv([1 0],[15 1]),[0.064 1]),[10.89 5.6 1]),[0.5184 1.4 1])
G11 = tf(5.305*num1,den1)
% Defining G22
num2 = conv(conv(conv([3.3 1],[2.1 1]),[0.4 1]),[6.25 5 1])
den2 = conv(conv(conv(conv(conv([1 0],[20 1]),[2.1 1]),[2.1 1]),[2.7 1]),[0.06 1])
G22 = tf(7.43*num2,den2)
% Overall controller matrix
Controller = [G11 0;0 G22]

% Defining GD12
GD12 = -P12/P11
% Defining GD21
GD21 = -P21/P22

% Loop 1 design
intermediate_L1 = P11-(P12*P21/P22)
L1 = intermediate_L1*G11
figure(2);
nichols(L1);
hold on
grid on
% Loop 2 design
intermediate_L2 = P22-(P21*P12/P11)
L2 = intermediate_L2*G22
nichols(L2);
grid on
hold off

%% Problem 3
%
tfinal = 30;
h = 0.01;
time = 0:h:tfinal;
sim problem3_simulink
figure(3)
subplot(4,1,1)
plot(time,Qsys,'linewidth',2)
title('SAWANT: MECE 7362 - Final Project - Problem 3 Simulink Output')
ylim([0 6])
grid on
ylabel('Systemic flow')
subplot(4,1,2)
plot(time,Qpul,'linewidth',2)
ylim([0 6])
grid on
ylabel('Pulmonary flow')
subplot(4,1,3)
plot(time,VL,'linewidth',2)
ylim([0 6])
grid on
ylabel('Left Pump Voltage')
subplot(4,1,4)
plot(time,VR,'linewidth',2)
ylim([0 5])
grid on
ylabel('Right Pump Voltage')
xlabel('Time')
%