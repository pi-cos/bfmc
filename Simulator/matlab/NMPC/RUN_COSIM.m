% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File Name: RUN_COSIM.m                                                  %
%                                                                         %
% Designed By: Enrico Picotti                                             %
% Company    : UniPD                                                      %
% Project    : Bosch RC Car                                               %
% Purpose    : runs the cosimulation of RC car BOSCH sim & NMPC control   %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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

setup.vel2motor = 1;
setup.rad2deg = rad2deg(1);

% !source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash
% !roslaunch sim_pkg map_with_car_nogui.launch
% roslaunch sim_pkg map_with_car.launch
% rosservice call /gazebo/reset_simulation --> NOT WORKING!!

%%

return

%% run simulink

sim('simulink_ros')

%% plots

draw_cosim;