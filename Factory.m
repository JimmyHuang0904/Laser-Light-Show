% This script sets all model  & controller parameters for the 
% Paint Booth Factory

% ==========
% INITIALIZE
% ==========
clear all;          % Erase everything from Matlab environment
load SolnPart1
CONSTANTS;          % Set physical constants
%load XYDesired
%DEFAULT;            %justine added this

SampleTime = 0.02;

% ============
% SYSTEM MODEL
% ============
% To display ideal robot trajectory:
%   - comment out the following two lines to use default values
%   - manually bypass all integrators in the Controller block

Robot;              % Declare parameters
Controller;         % Overwrite PID gains & throughput values
%Jacob; %Not using Jacob.m for now