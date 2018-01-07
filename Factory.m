% This script sets all model  & controller parameters for the 
% Paint Booth Factory

% ==========
% INITIALIZE
% ==========
clear all;        % Erase everything from Matlab environment
CONSTANTS;        % Set physical constants

% ============
% SYSTEM MODEL
% ============
% To display ideal robot trajectory:
%   - comment out the following two lines to use default values
%   - manually bypass all integrators in the Controller block

Controller;       % Overwrite PID gains & throughput values