
function xdot = AircraftEOM_doublet(time, aircraft_state, aircraft_surfaces, doublet_size,doublet_time, wind_inertial, aircraft_parameters)
    %% Doublet
    if time<doublet_time
        aircraft_surfaces(1)=aircraft_surfaces(1)+doublet_size;
    elseif time < 2*doublet_time
        aircraft_surfaces(1)=aircraft_surfaces(1)-doublet_size;
    end

    xdot=AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters);


end
