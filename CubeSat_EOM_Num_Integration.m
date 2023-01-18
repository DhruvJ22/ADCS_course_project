%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dhruv Jain
% 23 November 2021 
% Purdue University
%
% Obj: EOM setup for angular velocities and body angles
%       Time varying: Mass of Propellant, Cener of Mass, Principal Moment of Inertia
%       Moto misalignment and offset
%
% Angular velocities -> Euler's EOMs
% Angles -> Kinematic differential equaion using Body-three 3-1-2 rotation euler angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = CubeSat_EOM_Num_Integration(t,x, vars)
    
    % Decode CubeSat parameters
    mb = vars(1);
    l_ic_prop = vars(2); 
    l0 = vars(3);
    lt0 = vars(4);
    lt_dot = vars(5);
    r_motor = vars(6);
    mt0 = vars(7);
    mt_dot = vars(8);
    alpha = vars(9);
    d_offset = vars(10);
    Ibx = vars(11);
    Iby = vars(12);
    Ibz = vars(13);
    fz = vars(14);

    % Time varying Mass and C.M.
    if t < abs(mt0/mt_dot)
        lt = lt0 + lt_dot*t;
        mt = mt0 + mt_dot*t;
    else
        lt = lt0 + lt_dot*abs(mt0/mt_dot);
        mt = mt0 + mt_dot*abs(mt0/mt_dot);
    end

    h_noz_cm = ((l0+l_ic_prop)*mb + (l_ic_prop-lt)*mt)/(mb+mt);

    % Motor misalignment
    Mx = fz*(h_noz_cm*sind(alpha) + d_offset*cosd(alpha));
    
    % Time varying MOI
    Itx = mt*(r_motor^2/4 + lt^2/3);
    Ity = mt*(r_motor^2/4 + lt^2/3);
    Itz = mt*r_motor^2/2;
    
    Itxdot = mt_dot*(r_motor^2/4+lt^2/3) + 2/3*mt*lt*lt_dot;
    Itydot = mt_dot*(r_motor^2/4+lt^2/3) + 2/3*mt*lt*lt_dot;
    Itzdot = mt_dot*r_motor^2/2;

    Ix = Ibx + Itx + mt*(lt+l0)^2;
    Iy = Iby + Ity + mt*(lt+l0)^2;
    Iz = Ibz + Itz;

    Ixdot = Itxdot + mt_dot*(lt+l0)^2 + 2*mt*(lt+l0)*lt_dot;
    Iydot = Itydot + mt_dot*(lt+l0)^2 + 2*mt*(lt+l0)*lt_dot;
    Izdot = Itzdot;

    wx = x(1); wy = x(2); wz = x(3); px = x(4); py = x(5); pz = x(6); 
    
    % Dynamics - Euler's Equations of Motion
    dwx = (Mx + (Iy-Iz)*wy*wz - (Ixdot-mt_dot*(h_noz_cm^2 + d_offset^2))*wx)/Ix;
    dwy = ((Iz-Ix)*wz*wx - (Iydot - mt_dot*h_noz_cm^2)*wy)/Iy;
    dwz = ((Ix-Iy)*wx*wy - (Izdot - mt_dot*d_offset^2)*wz)/Iz;
    
    % Kinematics: Body-three 3-1-2 rotation sequence
    dpx = wx*cos(py)+wz*sin(py);
    dpy = wy-(wz*cos(py)-wx*sin(py))*tan(px);
    dpz = (wz*cos(py)-wx*sin(py))*sec(px);

    y = [dwx;dwy;dwz;dpx;dpy;dpz]; 
end