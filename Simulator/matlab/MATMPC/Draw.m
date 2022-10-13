%% Plot results

% define colors
blue=[0 0 255]/255;
red=[220 20 60]/255;
orange=[255 165 0]/255;
green=[0 205 102]/255;

% start generating pictures
switch settings.model
    
    case 'InvertedPendulum'
        
        figure(1);
        subplot(321)
        plot(time,state_sim(:,1));
        title('p');
        subplot(322)
        plot(time,state_sim(:,2)*180/pi);
        title('\theta');
        subplot(323)
        plot(time,state_sim(:,3));
        title('v');
        subplot(324)
        plot(time,state_sim(:,4)*180/pi);
        title('\omega');
        subplot(3,2,[5 6]);
        title('F');
        stairs(time,controls_MPC(:,1));

    case 'KinematicVehicle'

        figure(1)
        plot(state_sim(:,1),state_sim(:,2))
        xlabel('X [m]');
        ylabel('Y [m]');

        figure(2)
        subplot(211)
        plot(time,controls_MPC(:,1))
        xlabel('time [s]');
        subplot(212)
        plot(time,controls_MPC(:,2))
        xlabel('time [s]');
        ylabel('\delta_f [rad]');

end