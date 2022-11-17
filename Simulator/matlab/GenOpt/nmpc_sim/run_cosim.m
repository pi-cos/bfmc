function y = run_cosim(weights)
%RUN_COSIM Summary of this function goes here
%   Detailed explanation goes here

%% reset simulation

unix('source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; rosservice call /gazebo/reset_simulation');
clear nmpc_controller
pause(1);
open('nmpc_sim/cosim/simulink/simulink_ros.slx')

%% params

beta = 1; % parameter weighting e_v wrt e_y
WGN_LOC_PWR = 0;
DELAY_LOC = 0;
DELAY_VEL = 0;
DELAY_STEER = 0;

%% run

setup_solver

set_param('simulink_ros/WEIGHT1','Value',num2str(weights(1)))
set_param('simulink_ros/WEIGHT2','Value',num2str(weights(2)))
set_param('simulink_ros/WEIGHT3','Value',num2str(weights(3)))
set_param('simulink_ros/WEIGHT4','Value',num2str(weights(4)))
set_param('simulink_ros/WEIGHT5','Value',num2str(weights(5)))

assignin("base","WGN_LOC_PWR",WGN_LOC_PWR)
% set_param('simulink_ros/WGN_LOC_PWR','Value',num2str(WGN_LOC_PWR))
set_param('simulink_ros/DELAY_LOC','Value',num2str(DELAY_LOC))
set_param('simulink_ros/DELAY_VEL','Value',num2str(DELAY_VEL))
set_param('simulink_ros/DELAY_STEER','Value',num2str(DELAY_STEER))

out = sim('simulink_ros.slx');
finished_nom = out.nmpc_output.signals.values(end,9);
e_y_nom = out.nmpc_output.signals.values(:,6);
e_v_nom = out.nmpc_output.signals.values(:,8);

for ii = 1:10
    clear nmpc_controller
    WGN_LOC_PWR = 0;%1e-6*rand(1,1);
    DELAY_LOC = round(1+(5-1)*rand(1,1));
    DELAY_VEL = round(1+(5-1)*rand(1,1));
    DELAY_STEER = round(1+(5-1)*rand(1,1));
    assignin("base","WGN_LOC_PWR",WGN_LOC_PWR)
%     set_param('simulink_ros/WGN_LOC_PWR','Value',num2str(WGN_LOC_PWR))
    set_param('simulink_ros/DELAY_LOC','Value',num2str(DELAY_LOC))
    set_param('simulink_ros/DELAY_VEL','Value',num2str(DELAY_VEL))
    set_param('simulink_ros/DELAY_STEER','Value',num2str(DELAY_STEER))
    out = sim('simulink_ros.slx');
    finished(ii) = out.nmpc_output.signals.values(end,9);
    e_y_tot(ii) = mean(out.nmpc_output.signals.values(:,6));
    e_v_tot(ii) = mean(out.nmpc_output.signals.values(:,8));
end

if finished_nom && all(finished)
    y = [mean(e_y_nom)+beta*mean(e_v_nom),std(e_y_tot)+beta*std(e_v_tot)]; %var(e_y)
else
    y = [1e5;1e5];
end
pause(1);

end

