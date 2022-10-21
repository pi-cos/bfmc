% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File Name: RUN_SIM.m                                                    %
%                                                                         %
% Designed By: Enrico Picotti                                             %
% Company    : UniPD                                                      %
% Project    : Bosch RC Car                                               %
% Purpose    : runs the simulation of RC car NMPC controller              %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% clear ws

restoredefaultpath; clear all; clear mex; close all; clc;

%% path

addpath(genpath('init_fcn/'));
addpath('sim/');
addpath('plots/');

%% setup

setup_solver

%% simulation

% simulation_loop
simulation_loop_fcn

%% plots

Draw;



