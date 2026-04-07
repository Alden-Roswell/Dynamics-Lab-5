%Contributors: Robert Reynoso/Spencer M
%Course numer: ASEN 3801
%File Name: QuadrotorEOM
%created: 3/10/26

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
%inputs: state vector, gravity constant, mass, distance from CG to prop, control moment coeff, aero force coeff, aero moment coeff, motor forces
%outputs: use definitions to calculate derivative state vector 
    %% Unpacking

    xE    = var(1);
    yE    = var(2);
    zE    = var(3);

    phi   = var(4);   % roll
    theta = var(5);   % pitch
    psi   = var(6);   % yaw

    u     = var(7);
    v     = var(8);
    w     = var(9);

    p     = var(10);
    q     = var(11);
    r     = var(12);

    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

    f1 = motor_forces(1);
    f2 = motor_forces(2);
    f3 = motor_forces(3);
    f4 = motor_forces(4);

    Zc = -(f1 + f2 + f3 + f4);                         % total thrust (negative is up)
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);           % roll
    Mc = (d/sqrt(2)) * ( f1 - f2 - f3 + f4);           % pitch
    Nc = km             * ( f1 - f2 + f3 - f4);         % yaw

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
    Va = sqrt(u^2 + v^2 + w^2);
    Xaero = -nu * Va * u;          % drag force x
    Yaero = -nu * Va * v;          % drag force y 
    Zaero = -nu * Va * w;          % drag force z 
    Laero = -mu * abs(p) * p;   % drag moment about body x (opposes roll  rate)
    Maero = -mu * abs(q) * q;   % drag moment about body y (opposes pitch rate)
    Naero = -mu * abs(r) * r;   % drag moment about body z (opposes yaw   rate)

    %% Dynamics

    u_dot = r*v - q*w - g*sth          + Xaero/m;
    v_dot = p*w - r*u + g*cth*sphi     + Yaero/m;
    w_dot = q*u - p*v + g*cth*cphi     + Zaero/m + Zc/m;

    p_dot = ((Iy - Iz)/Ix)*q*r + (Laero + Lc)/Ix;
    q_dot = ((Iz - Ix)/Iy)*p*r + (Maero + Mc)/Iy;
    r_dot = ((Ix - Iy)/Iz)*p*q + (Naero + Nc)/Iz;

    %% Assemble
    var_dot = [xE_dot;
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
