%% Plot results

% define colors
blue=[0 0 255]/255;
red=[220 20 60]/255;
orange=[255 165 0]/255;
green=[0 205 102]/255;

% start generating pictures
switch settings.model
    
%     case 'InvertedPendulum'
%         
%         figure(1);
%         subplot(321)
%         plot(time,state_sim(:,1));
%         title('p');
%         subplot(322)
%         plot(time,state_sim(:,2)*180/pi);
%         title('\theta');
%         subplot(323)
%         plot(time,state_sim(:,3));
%         title('v');
%         subplot(324)
%         plot(time,state_sim(:,4)*180/pi);
%         title('\omega');
%         subplot(3,2,[5 6]);
%         title('F');
%         stairs(time,controls_MPC(:,1));

    case 'KinematicVehicle'

        figure(1)
        hold on
        grid on
        plot(state_sim(:,1),state_sim(:,2))
        R = 1/input.od(1);
        plot(track.X,track.Y,'k--');
        axis equal
        xlabel('X [m]');
        ylabel('Y [m]');

        figure(2)
        subplot(211)
        plot(time(1:size(controls_MPC,1)),controls_MPC(:,1))
        grid on
        xlabel('space [m]');
        ylabel('$\dot{v} [m/s^2]$','Interpreter','latex')
        subplot(212)
        plot(time(1:size(controls_MPC,1)),rad2deg(controls_MPC(:,2)))
        grid on
        xlabel('space [m]');
        ylabel('$\dot{\delta}_f [deg/s]$','Interpreter','latex');

        figure(3)
        subplot(211)
        hold on
        grid on
        plot(time(1:size(state_sim,1)),state_sim(:,4))
        plot(time(1:size(computed_errors,1)),computed_errors(:,1),'--')
        xlabel('space [m]');
        ylabel('e_y [m]')
        legend('state sim','computed')
        subplot(212)
        hold on
        grid on
        plot(time(1:size(state_sim,1)),rad2deg(state_sim(:,5)))
        plot(time(1:size(computed_errors,1)),rad2deg(computed_errors(:,2)),'--')
        xlabel('space [m]');
        ylabel('e_\psi [deg]');
        legend('state sim','computed')

        figure(4)
        subplot(211)
        plot(time(1:size(state_sim,1)),state_sim(:,6))
        grid on
        xlabel('space [m]');
        ylabel('$v [m/s]$','Interpreter','latex')
        subplot(212)
        plot(time(1:size(state_sim,1)),rad2deg(state_sim(:,7)))
        grid on
        xlabel('space [m]');
        ylabel('$\delta_f [deg]$','Interpreter','latex');

end