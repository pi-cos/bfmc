%% 
restoredefaultpath; clear all; close all; clc; %bdclose();


%% launch simulation

disp('Launch the following command on terminal to start the simulation the first time:')
disp(' ')
disp(['source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; '...
    'roslaunch sim_pkg map_with_car_nogui.launch'])

unix('source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; rosservice call /gazebo/reset_simulation');

%% set path

addpath('/home/pico/Documents/GitHub/bfmc/Simulator/src/matlab_msg_gen_ros1/glnxa64/install/m')
addpath(genpath('nmpc_sim'));

%% setup

setup_solver


setup.vel2motor = 1/1.4;
setup.rad2deg = rad2deg(1);
setup.stop_time = inf; %[s]

setup.sim_ts = 1e-2;

%% open simulink

open('nmpc_sim/cosim/simulink/simulink_ros.slx')
% run_cosim([1 2 3 4 5]);

%% set genetic algorithm

FitnessFunction = @run_cosim;
numberOfVariables = 5;
lb = [1e-2;1e-2;1e-5;1e-5;1e-5];
ub = [1e+3;1e+3;1e+1;1e+1;1e+1];

options = optimoptions(@gamultiobj,'PlotFcn',{@gaplotpareto},'MaxTime',12*60*60);

%% run genetic algorithm

[x,fval] = gamultiobj(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub,options);

save simulink_cosim_results
