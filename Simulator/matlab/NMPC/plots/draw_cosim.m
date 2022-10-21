%% Plot results

%%

figure(1)
hold on
grid on
plot(out.state_cosim(:,1)-out.state_cosim(1,1),out.state_cosim(:,2)-out.state_cosim(1,2))
R = 1/input.od(1);
plot(track.X,track.Y,'k--');
axis equal
xlabel('X [m]');
ylabel('Y [m]');

%%

figure(2)
subplot(211)
grid on
hold on
plot(controls_MPC(2:end,1))
plot(out.controls(:,1),'--')
% xlabel('space [m]');
ylabel('v [m/s]')
subplot(212)
grid on
hold on
plot(rad2deg(controls_MPC(2:end,2)))
plot(rad2deg(out.controls(:,2)),'--')
% xlabel('space [m]');
ylabel('\delta_f [deg]');

%%

figure(3)
subplot(211)
hold on
grid on
plot(computed_errors(:,1),'--')
% xlabel('space [m]');
ylabel('e_y [m]')
subplot(212)
hold on
grid on
plot(rad2deg(computed_errors(:,2)),'--')
% xlabel('space [m]');
ylabel('e_\psi [deg]');
