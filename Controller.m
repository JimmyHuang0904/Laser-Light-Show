
% =================
% Rotation Matrices
% =================

R0_1 = [[cos(Theta1), -sin(Theta1), 0], [0, 0, -1],[sin(Theta1), cos(Theta1), 0]];
R1_2 = [[cos(Theta2), -sin(Theta2), 0], [sin(Theta2), cos(Theta2), 0], [0, 0, 1]];

% ====================
% Displacement Vectors
% ====================

D0_1 = [0, 0, Height1];
D1_2 = [Length2*cos(Theta2), Length2*sin(Theta2), 0];
