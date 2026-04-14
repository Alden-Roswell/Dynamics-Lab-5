clear;
close all;
clc;


run('ttwistor.m');
aircraft_surfaces = [0,0,0,0]';
wind_intertial = [0,0,0]';
y0 = [0 0 -1609.4 0 0 0 21 0 0 0 0 0]';
tspan = [0,100];
ode = @(time, aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_intertial, aircraft_parameters);
[t,X] = ode45(ode,tspan,y0);
control_array = zeros(4, length(X));
fig = 1:1:6
col = ["b", "asd"]
PlotAircraftSim(t,X', control_array,fig, col)


aircraft_surfaces_2 = [0.1079,0,0,0.3181]';
wind_intertial_2 = [0,0,0]';
            
y0_2 = [0 0 -1800  ...
        0 0.0278 0    ...
        20.99 0 0.5837 ...
        0 0 0]';
tspan_2 = [0,100];
ode_2 = @(time, aircraft_state_2) AircraftEOM(time, aircraft_state_2, aircraft_surfaces_2, wind_intertial_2, aircraft_parameters);
[t_2,X_2] = ode45(ode_2,tspan_2,y0_2);
control_array_2 = repmat(aircraft_surfaces_2', length(X_2),1)';
fig_2 = 7:1:12
col = ["b", "asd"]
PlotAircraftSim(t_2,X_2', control_array_2,fig_2, col)



aircraft_surfaces_21 = deg2rad([5,2,-13,0.3])';
wind_intertial_21 = [0,0,0]';
            
y0_21 = [0 0 -1800  ...
        deg2rad([15, -12, 270])    ...
        19 3 -2 ...
        deg2rad([0.08 -0.2 0])]';
tspan_21 = [0,100];
ode_21 = @(time, aircraft_state_21) AircraftEOM(time, aircraft_state_21, aircraft_surfaces_21, wind_intertial_21, aircraft_parameters);
[t_21,X_21] = ode45(ode_21,tspan_21,y0_21);
control_array_21 = repmat(aircraft_surfaces_21', length(X_21),1)';
fig_21 = 7:1:12
col = ["r", "asd2"]
PlotAircraftSim(t_21,X_21', control_array_21,fig_21, col)
axis equal