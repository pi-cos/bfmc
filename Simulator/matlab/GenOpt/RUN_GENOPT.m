%% 
restoredefaultpath; clear all; close all; clc;


%% launch simulation

disp('Launch the following command on terminal to start the simulation the first time:')
disp(' ')
disp(['source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; '...
    'roslaunch sim_pkg map_with_car_nogui.launch'])
disp(' ')

unix('source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; rosservice call /gazebo/reset_simulation');

%% set path

addpath('/home/pico/Documents/GitHub/bfmc/Simulator/src/matlab_msg_gen_ros1/glnxa64/install/m')
addpath(genpath('nmpc_sim'));
addpath('utils')

%% setup


setup.vel2motor = 1/1.4;
setup.rad2deg = rad2deg(1);
setup.stop_time = 30; %[s]

setup.sim_ts = 1e-2;

setup_solver

%% open simulink

disp('Opening Simulink..')

open('nmpc_sim/cosim/simulink/simulink_ros.slx')

%% single run & return

% load partial_ga_results.mat
% run_cosim([10 10 10 0.1 0.1]);
% run_cosim(gapopulationhistory(45,:));
% cosim_res = run_cosim(pop_history(1,:,end));
% return

%% set genetic algorithm

FitnessFunction = @run_cosim;
numberOfVariables = 5;
lb = [1e-1;1e-1;1e-1;1e-3;1e-3];
ub = [1e+3;1e+3;1e+1;1e1;1e1];

options = optimoptions(@gamultiobj,'PlotFcn',{@gaplotpareto},'Display','iter', 'OutputFcns', @gaoutfun,'MaxGenerations',50); %'MaxTime',48*60*60,

%% run genetic algorithm

[x,fval,exitflag,output] = gamultiobj(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub,options);

save simulink_cosim_results
