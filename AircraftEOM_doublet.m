
function xdot = AircraftEOM_doublet(time, aircraft_state, aircraft_surfaces, doublet_size,doublet_time, wind_inertial, aircraft_parameters)
    %% Doublet
    delta_e_t=aircraft_state(1);
    if time<doublet_time
        delta_e=delta_e_t+doublet_size;
    elseif time < 2*doublet_time
        delta_e=delta_e_t-doublet_size;
    else
        delta_e=delta_e_t;
    end
    aircraft_state(1)=delta_e;

    xdot=AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters);


end
