clear;
close all;
clc;


run('ttwistor.m');
aircraft_surfaces = [0,0,0,0]';
wind_intertial = [0,0,0]';
y0 = [0 0 1609.4 21 0 0 0 0 0 0 0 0]';
tspan = [0,20];
ode = @(time, aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_intertial, aircraft_parameters);
[t,X] = ode45(ode,tspan,y0);

