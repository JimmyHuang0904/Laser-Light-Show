
% =================
%    PID Values
% =================

KD0 = 0.0009;
KD1 = 1;

Nvalue = 100;

PID0=[0.731*KD0     0*KD1       1*KD0     Nvalue];
PID1=[205*KD1       0.01*KD1    1*KD1     Nvalue];

% =================
% Rotation Matrices
% =================

%R0_1 = [[cos(Theta1), -sin(Theta1), 0], [0, 0, -1],[sin(Theta1), cos(Theta1), 0]];
%R1_2 = [[cos(Theta2), -sin(Theta2), 0], [sin(Theta2), cos(Theta2), 0], [0, 0, 1]];

% ====================
% Displacement Vectors
% ====================

%D0_1 = [0, 0, Height];
%D1_2 = [Length2*cos(Theta2), Length2*sin(Theta2), 0];

% ====================
% Mech Dynamics
% ====================

Mech0n = [1];
Mech0d = [1];

Mech1n = [1];
Mech1d = [1];

% ====================
% Elec Dynamics
% ====================

Elec1n = [1];
Elec1d = [1];

% ====================
% Gains
% ====================

Sens0 = [1];
Sens1 = [1];
FB1 = [1];

% ====================
% Xd, Yd
% ====================


Conversion = 1.8/2*pi/180*1000;

%zeta = 1;
%w0 = (4.65*zeta-1.3)/0.300; % overdamped
%w1 = (2.16*zeta+0.6)/0.300;
%Kfinal = 500;
zeta = 0.41;
Kfinal = 400;
w0 = (2.16*zeta+0.6)/(0.104);

H = tf(Kfinal*w0^2, [1, 2*zeta*w0, w0^2])
Z = tf(Kfinal*w1^2, [1, 2*zeta*w1, w1^2]);

Mech0ntest = [Kfinal*w0^2];
Mech0dtest = [1 2*zeta*w0 w0^2];



stepplot(H);
%stepplot(Z);
grid on;
grid minor;