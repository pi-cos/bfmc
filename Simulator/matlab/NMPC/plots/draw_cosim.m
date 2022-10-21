%% Plot results

%%

figure(1)
hold on
grid on
plot(out.state_cosim.signals.values(:,1),out.state_cosim.signals.values(:,2))
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
% plot(controls_MPC(2:end,1))
plot(out.controls.time,out.controls.signals.values(:,1))
% xlabel('time [s]');
ylabel('motor speed')
subplot(212)
grid on
hold on
% plot(rad2deg(controls_MPC(2:end,2)))
plot(out.controls.time,out.controls.signals.values(:,2))
xlabel('time [s]');
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
xlabel('iter [-]');
ylabel('e_\psi [deg]');
