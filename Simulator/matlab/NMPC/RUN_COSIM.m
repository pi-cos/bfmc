% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File Name: RUN_COSIM.m                                                  %
%                                                                         %
% Designed By: Enrico Picotti                                             %
% Company    : UniPD                                                      %
% Project    : Bosch RC Car                                               %
% Purpose    : RUNS the cosimulation of RC car BOSCH sim & NMPC control   %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% clear ws

restoredefaultpath; clear all; clear mex; close all; clc;

%% launch simulation

% [status, cmdout] = unix('source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; roslaunch sim_pkg map_with_car_nogui.launch');
% unix('source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; roslaunch sim_pkg map_with_car_nogui.launch')
% pause(2);

% note: rosservice call /gazebo/reset_simulation --> NOT WORKING!!

disp('Launch the following command on terminal:')
disp(' ')
disp(['source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; '...
    'roslaunch sim_pkg map_with_car_nogui.launch'])
disp(' ')
disp('then press enter to start the cosimulation.')
pause()

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

%% run simulink

disp(' ')
disp('Starting simulink controller...')
out = sim('simulink_ros');

%% plots

draw_cosim;

%% terminate cosim

disp(' ')
disp('Press CTRL+C in terminal to stop simulatior.')