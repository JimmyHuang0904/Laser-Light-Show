
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

R0_1 = [[cos(Theta1), -sin(Theta1), 0], [0, 0, -1],[sin(Theta1), cos(Theta1), 0]];
R1_2 = [[cos(Theta2), -sin(Theta2), 0], [sin(Theta2), cos(Theta2), 0], [0, 0, 1]];

% ====================
% Displacement Vectors
% ====================

D0_1 = [0, 0, Height];
D1_2 = [Length2*cos(Theta2), Length2*sin(Theta2), 0];

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

Xd = [3];
Yd = [3];
Time = [1];