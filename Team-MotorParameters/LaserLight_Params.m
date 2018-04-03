% This script sets all model  & controller parameters for the 
% Paint Booth Factory

% ==========
% INITIALIZE
% ==========
clear all;          % Erase everything from Matlab environment
%load SolnPart1
CONSTANTS;          % Set physical constants
%load XYDesired
%DEFAULT;            %only need if want to use defaut values

SampleTime = 0.02;
D=10; 

% ============
% SYSTEM MODEL
% ============
% To display ideal robot trajectory:
%   - comment out the following two lines to use default values
%   - manually bypass all integrators in the Controller block

System;              % Declare system parameters
Control;         % Overwrite PID gains & throughput values
                    %Jacob; %Not using Jacob.m for now