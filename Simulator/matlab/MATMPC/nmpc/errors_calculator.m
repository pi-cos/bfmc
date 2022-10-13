function [e_y, e_psi] = errors_calculator(current, curr_index, traj)
%ERRORS_CALCULATOR Computes the errors e_y, e_psi
%   Detailed explanation goes here
%   Written by Enrico Picotti in date 03/01/2018

%% init

psi_ref = traj.psi(curr_index);

%% Compute e_psi
e_psi = current.psi - psi_ref;
% if (e_psi > 2*pi)
%     e_psi = e_psi - 2*pi;
% elseif (e_psi < -pi)
%     e_psi = e_psi + 2*pi;
% end

while (e_psi > 2*pi)
    e_psi = e_psi - 2*pi;
end
while (e_psi < -pi)
    e_psi = e_psi + 2*pi;
end

%% Compute e_y

e_y_old = cos(psi_ref+pi)*((-current.Y)-(-traj.Y(curr_index)))-sin(psi_ref+pi)*((-current.X)-(-traj.X((curr_index))));

e_y = cos(psi_ref)*(current.Y-(traj.Y(curr_index)))-sin(psi_ref)*(current.X-(traj.X(curr_index)));

err = abs(e_y-e_y_old);
if err > 1e-3
    warning(['Check error computations, old - new difference! Error magnitude = ', num2str(err)])
end

error_angle = psi_ref + pi/2;
pos_computed = [traj.X(curr_index); traj.Y(curr_index)] + [e_y*cos(error_angle); e_y*sin(error_angle)];

if curr_index<length(traj.X) && curr_index ~= 1
    err = sqrt((pos_computed(1)-current.X)^2 + (pos_computed(2)-current.Y)^2);
    if err > ...
        sqrt((traj.X(curr_index)-traj.X(curr_index+1))^2 + (traj.Y(curr_index)-traj.Y(curr_index+1))^2)
        warning(['Check error computations! Difference in pos computed and current. Error magnitude = ', num2str(err), ...
                 ' * Current [X,Y,yaw] = [', num2str(current.X),',',num2str(current.Y),',',num2str(rad2deg(current.psi)),']',...
                 ' * Trajectory [X,Y,yaw] = [', num2str(traj.X(curr_index)),',',num2str(traj.Y(curr_index)),',',num2str(rad2deg(traj.psi(curr_index))),']',...
                 ' * Errors [ey,epsi] = [', num2str(e_y),',',num2str(e_psi),']'])
        
    end 
end

end

