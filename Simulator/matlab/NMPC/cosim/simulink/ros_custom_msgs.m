clear all; close all; clc

folderpath = '/home/pico/Documents/GitHub/bfmc/Simulator/matlab/rosmsgs/utils';
rosgenmsg('/home/pico/Documents/GitHub/bfmc/Simulator/matlab/rosmsgs')

%%

addpath('/home/pico/Documents/GitHub/bfmc/Simulator/matlab/rosmsgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear classes
rehash toolboxcache
rosmsg list

%%

addpath('/home/pico/Documents/GitHub/bfmc/Simulator/src/matlab_msg_gen_ros1/glnxa64/install/m')

clear classes
rehash toolboxcache

rosmsg list