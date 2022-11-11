function y = run_cosim(weights)
%RUN_COSIM Summary of this function goes here
%   Detailed explanation goes here

%% reset simulation

unix('source /home/pico/Documents/GitHub/bfmc/Simulator/devel/setup.bash; rosservice call /gazebo/reset_simulation');
clear nmpc_controller
bdclose();
clear nmpc_controller
pause(1);
open('nmpc_sim/cosim/simulink/simulink_ros.slx')

%%

setup_solver

set_param('simulink_ros/WEIGHT1','Value',num2str(weights(1)))
set_param('simulink_ros/WEIGHT2','Value',num2str(weights(2)))
set_param('simulink_ros/WEIGHT3','Value',num2str(weights(3)))
set_param('simulink_ros/WEIGHT4','Value',num2str(weights(4)))
set_param('simulink_ros/WEIGHT5','Value',num2str(weights(5)))
out = sim('simulink_ros.slx');
finished = out.nmpc_output.signals.values(end,9);
e_y = out.nmpc_output.signals.values(:,6);
e_v = out.nmpc_output.signals.values(:,8);
if finished
    y = [mean(e_y);mean(e_v)]; %var(e_y)
else
    y = [1e10;1e10];
end
pause(1);

end

