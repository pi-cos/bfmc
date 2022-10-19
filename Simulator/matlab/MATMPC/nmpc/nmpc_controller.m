function [ctrls] = nmpc_controller(states)
%NMPC_CONTROLLER Computes the controls for the RC car
%   Inputs:
%        - states: contains car position and velocities from localisation
%   Output:
%        - ctrls: contains steering and velocity inputs fro RC car

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File Name: nmpc_controller.m                                            %
%                                                                         %
% Designed By: Enrico Picotti                                             %
% Company    : UniPD                                                      %
% Project    : Bosch RC Car                                               %
% Purpose    : Implements the NMPC controller                             %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

persistent iter last_index 
persistent ref settings config opt mem input stats gear_prec timeTicEVO


outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

