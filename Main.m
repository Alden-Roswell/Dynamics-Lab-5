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