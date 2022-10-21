%% Plot results

%%

figure(1)
hold on
grid on
plot(state_cosim(:,1),state_cosim(:,2))
R = 1/input.od(1);
plot(track.X,track.Y,'k--');
axis equal
xlabel('X [m]');
ylabel('Y [m]');

figure(2)
subplot(211)
plot(time,controls_MPC(:,1))
grid on
xlabel('space [m]');
ylabel('v [m/s]')
subplot(212)
plot(time,rad2deg(controls_MPC(:,2)))
grid on
xlabel('space [m]');
ylabel('\delta_f [deg]');

figure(3)
subplot(211)
hold on
grid on
plot(time,computed_errors(:,1),'--')
xlabel('space [m]');
ylabel('e_y [m]')
subplot(212)
hold on
grid on
plot(time,rad2deg(computed_errors(:,2)),'--')
xlabel('space [m]');
ylabel('e_\psi [deg]');
