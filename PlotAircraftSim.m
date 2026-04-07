function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

%%Inertial Position
figure(fig(1));
hold on;
subplot(3,1,1); plot(time, aircraft_state_array(1,:),col(1),DisplayName=col(2))
title('Inertial Position: x (North)')
xlabel('Time (s)')
ylabel('x (m)')
legend('location','best')
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(2,:),col(1),DisplayName=col(2))
title('Inertial Position: y (East)')
xlabel('Time (s)')
ylabel('y (m)')
legend('location','best')
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(3,:),col(1),DisplayName=col(2))
title('Inertial Position: z (Down)')
xlabel('Time (s)')
ylabel('z (m)')
set(gca, 'YDir', 'reverse')
legend('location','best')
grid on;
hold on;


%%Euler Angles
figure(fig(2));
hold on;
subplot(3,1,1); plot(time, aircraft_state_array(4,:),col(1),DisplayName=col(2))
title('Euler Angles: \phi (Roll)')
xlabel('Time (s)')
ylabel('\phi (rad)')
legend('location','best')
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(5,:),col(1),DisplayName=col(2))
title('Euler Angles: \theta (Pitch)')
xlabel('Time (s)')
ylabel('\theta (rad)')
legend('location','best')
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(6,:),col(1),DisplayName=col(2))
title('Euler Angles: \psi (Yaw)')
xlabel('Time (s)')
ylabel('\psi (rad)')
legend('location','best')
grid on;
hold on;

%%Inertial Body Frame Velocity
figure(fig(3));
hold on;
subplot(3,1,1); plot(time, aircraft_state_array(7,:),col(1),DisplayName=col(2))
title('Body Frame Inertial Velocity: u (Forward)')
xlabel('Time (s)')
ylabel('u (m/s)')
legend('location','best')
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(8,:),col(1),DisplayName=col(2))
title('Body Frame Inertial Velocity: v (Right)')
xlabel('Time (s)')
ylabel('v (m/s)')
legend('location','best')
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(9,:),col(1),DisplayName=col(2))
title('Body Frame Inertial Velocity: w (Down)')
xlabel('Time (s)')
ylabel('w (m/s)')
set(gca, 'YDir', 'reverse')
legend('location','best')
grid on;
hold on;

figure(fig(4));
hold on;
%%Angular Velocity
subplot(3,1,1); plot(time, aircraft_state_array(10,:),col(1),DisplayName=col(2))
title('Angular Velocity: p (Roll Rate)')
xlabel('Time (s)')
ylabel('p (rad/s)')
legend('location','best')
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(11,:),col(1),DisplayName=col(2))
title('Angular Velocity: q (Pitch Rate)')
xlabel('Time (s)')
ylabel('q (rad/s)')
legend('location','best')
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(12,:),col(1),DisplayName=col(2))
title('Angular Velocity: r (Yaw Rate)')
xlabel('Time (s)')
ylabel('r (rad/s)')
legend('location','best')
grid on;
hold on;

figure(fig(5))
hold on;
grid on;
subplot(4,1,1); plot(time, rad2deg(control_input_array(1,:)),col(1),DisplayName=col(2))
title('Control Input: Elevator')
xlabel('Time (s)')
ylabel('Z_c')
set(gca, 'YDir', 'reverse')
legend('location','best')
grid on;
hold on;

subplot(4,1,2); plot(time, rad2deg(control_input_array(2,:)),col(1),DisplayName=col(2))
title('Control Input: Aileron')
xlabel('Time (s)')
ylabel('L_c')
legend('location','best')
grid on;
hold on;

subplot(4,1,3); plot(time, rad2deg(control_input_array(3,:)),col(1),DisplayName=col(2))
title('Control Input: Rudder')
xlabel('Time (s)')
ylabel('M_c')
legend('location','best')
grid on;
hold on;

subplot(4,1,4); plot(time, control_input_array(4,:),col(1),DisplayName=col(2))
title('Control Input: Throttle')
xlabel('Time (s)')
ylabel('N_c')
legend('location','best')
grid on;
hold on;

pEout = aircraft_state_array(1:3,:);
pEmins = min(pEout,[],2);
pEmaxs = max(pEout,[],2);
pErange = pEmaxs-pEmins;
pErange = 0.5*ones(3,1)*max(pErange);

figure(fig(6));
hold on;
grid on;
plot3(aircraft_state_array(1,:),aircraft_state_array(2,:),aircraft_state_array(3,:),col(1),'DisplayName',col(2));
scatter3(aircraft_state_array(1,1),aircraft_state_array(2,1),aircraft_state_array(3,1), 'g', 'filled','HandleVisibility','off')
scatter3(aircraft_state_array(1,end),aircraft_state_array(2,end),aircraft_state_array(3,end), 'r', 'filled','HandleVisibility','off')
set(gca, 'ZDir', 'reverse')
axis equal; box on;
%try xlim([pEmins(1)-pErange(1) pEmaxs(1)+pErange(1)]); catch; end
%try ylim([pEmins(2)-pErange(2) pEmaxs(2)+pErange(2)]); catch; end
%try zlim([pEmins(3)-pErange(3) pEmaxs(3)+pErange(3)]); catch; end
view(3);
% legend('Position',[0.762452689691095 0.599999999354283 0.12578125141561 0.0322916673123835]);
legend('location','best');
xlabel('North (m)');
ylabel('East (m)');
zlabel('Down (m)');
end