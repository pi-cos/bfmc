% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File Name: RUN_COSIM.m                                                  %
%                                                                         %
% Designed By: Enrico Picotti                                             %
% Company    : UniPD                                                      %
% Project    : Bosch RC Car                                               %
% Purpose    : INIT the cosimulation of RC car BOSCH sim & NMPC control   %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% to run the cosimulation:
% 1: in a terminal run "source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash"
% 2: in the same terminal run "roslaunch sim_pkg map_with_car.launch"
% 3: run the simulink simulink_ros.slx

% rosservice call /gazebo/reset_simulation --> NOT WORKING!!

%% clear ws

restoredefaultpath; clear all; clear mex; close all; clc;

%% path

addpath('/home/pico/Documents/GitHub/bfmc/Simulator/src/matlab_msg_gen_ros1/glnxa64/install/m')
addpath(genpath('init_fcn/'));
addpath(genpath('cosim/'));
addpath('plots/');

%% setup

setup_solver

%% cosim params

setup.vel2motor = 1/1.4;
setup.rad2deg = rad2deg(1);

%%

return

%% run simulink

sim('simulink_ros')

%% plots

draw_cosim;