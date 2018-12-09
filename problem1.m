%%%%%  MECE 7362 Robust Multivariable Control  %%%%%
%%%%%        Final Project - Problem 1         %%%%%
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

% The amplitude of the off diagonal terms is less than the amplitude
% of the diagonal term. Therefore the given system is diagonally dominant.

% Defining G11
G11 = tf(2.308,[1 0])
% Defining G22
G22 = tf(1.7299,[1 0])
% Overall controller matrix
Controller = [G11 0;0 G22]

% Loop 1 design
L1 = G11*P11
figure(2);
nichols(L1);
hold on
grid on
% Loop 2 design
L2 = G22*P22
nichols(L2);
grid on
hold off

%% Problem 1
%
tfinal = 30;
h = 0.01;
time = 0:h:tfinal;
sim problem1_simulink
figure(3)
subplot(4,1,1)
plot(time,Qsys,'linewidth',2)
title('SAWANT: MECE 7362 - Final Project - Problem 1 Simulink Output')
ylim([0 6])
grid on
ylabel('Systemic flow')
subplot(4,1,2)
plot(time,Qpul,'linewidth',2)
ylim([0 8])
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