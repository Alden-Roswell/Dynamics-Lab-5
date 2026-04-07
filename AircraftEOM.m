
function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
%inputs: state vector, gravity constant, mass, distance from CG to prop, control moment coeff, aero force coeff, aero moment coeff, motor forces
%outputs: use definitions to calculate derivative state vector 
    %% Unpacking
    g = aircraft_parameters.g;
    m = aircraft_parameters.m;
    xE    = aircraft_state(1);
    yE    = aircraft_state(2);
    zE    = aircraft_state(3);

    phi   = aircraft_state(4);   % roll
    theta = aircraft_state(5);   % pitch
    psi   = aircraft_state(6);   % yaw

    u     = aircraft_state(7);
    v     = aircraft_state(8);
    w     = aircraft_state(9);

    p     = aircraft_state(10);
    q     = aircraft_state(11);
    r     = aircraft_state(12);

    Ix = aircraft_parameters.Ix;
    Iy = aircraft_parameters.Iy;
    Iz = aircraft_parameters.Iz;

    %% Aero Forces
    
    [~,~,~,density] = atmosisa(-zE);
    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
    L = aero_moments(1);
    M = aero_moments(2);
    N = aero_moments(3);
    X = aero_forces(1);
    Y = aero_forces(2);
    Z = aero_forces(3);

    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    sth  = sin(theta);
    cpsi = cos(psi);
    spsi = sin(psi);
    tth  = tan(theta); 

    %% Kinematics

    xE_dot =  (cth*cpsi)*u + (sphi*sth*cpsi - cphi*spsi)*v + (cphi*sth*cpsi + sphi*spsi)*w;
    yE_dot =  (cth*spsi)*u + (sphi*sth*spsi + cphi*cpsi)*v + (cphi*sth*spsi - sphi*cpsi)*w;
    zE_dot = -(sth)     *u + (sphi*cth)                 *v + (cphi*cth)                 *w;

    phi_dot   = p + sphi*tth*q + cphi*tth*r;
    theta_dot =     cphi*q     - sphi*r;
    psi_dot   =    (sphi/cth)*q + (cphi/cth)*r;

    %% Drag Stuff
   
    %% Dynamics

    u_dot = r*v - q*w - g*sth          + X/m;
    v_dot = p*w - r*u + g*cth*sphi     + Y/m;
    w_dot = q*u - p*v + g*cth*cphi     + Z/m;

    p_dot = ((Iy - Iz)/Ix)*q*r + (L)/Ix;
    q_dot = ((Iz - Ix)/Iy)*p*r + (M)/Iy;
    r_dot = ((Ix - Iy)/Iz)*p*q + (N)/Iz;

    %% Assemble
    xdot = [xE_dot;
               yE_dot;
               zE_dot;
               phi_dot;
               theta_dot;
               psi_dot;
               u_dot;
               v_dot;
               w_dot;
               p_dot;
               q_dot;
               r_dot];
end
