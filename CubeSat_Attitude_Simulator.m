%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dhruv Jain
% 23 November 2021 
% Purdue University
%
% Obj: To perform an analysis on the effect of Motor Misalignment,
%      Jet-Damping and Mass Variation on a Spinning Thrusting CubeSat
%      CubeSat's Angular Velocity and Orientation are computed using:
%       1. Numerical integration of EOMs
%       2. Analytical computation of simplified EOMs 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear
% clc 

%% Initalization
mb = 3;%kg, Dry mass of Cubesat 
l_ic_prop = 0.045;%m, Initial length of propellant
l0 = 0.15;%m, Distance between OT and O
lt0 = 0.0225;%m, DIstance between OT and tip of propellant
ltdot = -0.0056;%m/s, Rate change of lt0 with time

mt0 = 0.1;%kg, Initial mass of propellant
mtdot = 1e-15 + -0.025;%kg/s, Rate change of mass of propellant with respect to time
r_motor = 0.01;%m, Radius of propellant tank

alpha = 0.25; %deg, Misalignment angle of motor
d_offset = 0.001; % Motor Offset from C.M.

Ibx = 0.035; %kg-m^2, MOI about Body x-axis
Iby = 0.035; %kg-m^2, MOI about Body y-axis
Ibz = 0.007; %kg-m^2, MOI about Body z-axis

fz = 30;% Body-fixed initial force about z-axis

wz0_ic = 25; % rad/s, I.C. of angular velocity about body z-axis

tspan = [0 7]; % Simulation Time

ic = [0 0 wz0_ic 0 0 0]; % Initial condition, wx0, wy0, wz0, phix0, phiy0, phiz0

% Numerical Simulation
opt1 = odeset('RelTol',1e-12,'AbsTol',1e-12);
vars = [mb l_ic_prop l0 lt0 ltdot r_motor mt0 mtdot alpha d_offset Ibx Iby Ibz fz];
[tint, yv] = ode45(@d, tspan, ic, opt1, vars);
[Ix, Iy, Iz] = Ivals(tint,vars);

% Analytical Solution 
nutation = atan(sqrt((Ix.*yv(:,1)).^2+(Iy.*yv(:,2)).^2)./(Iz.*yv(:,3)));
[wx_analytic, wy_analytic, phi_x_analytic, phi_y_analytic,phi_z_analytic] = analytic_vals(tint, vars,yv);
wz_analytic = wz0_ic*ones(length(wx_analytic),1);

% Angular Momentum Components
Hx = Ix.*yv(:,1);
Hy = Iy.*yv(:,2);
Hz = Iz.*yv(:,3);

%% Plot
pltnum = 1;

figure(pltnum)
plot(tint,yv(:,1)-wx_analytic)
hold on
plot(tint,yv(:,2)-wy_analytic)
plot(tint,yv(:,3)-wz_analytic)
xlabel('t (sec)')
ylabel('\Delta w_i(t) rad/s')
grid on
legend('\Delta w_x','\Delta w_y','\Delta w_z','location','best')
pltnum = pltnum +1;

figure(pltnum)
plot(tint,yv(:,4)-phi_x_analytic)
hold on
plot(tint,yv(:,5)-phi_y_analytic)
plot(tint,yv(:,6)-phi_z_analytic)
xlabel('t (sec)')
ylabel('\Delta \phi_i(t) rad')
grid on
legend('\Delta \phi_x','\Delta \phi_y','\Delta \phi_z','location','best')
pltnum = pltnum +1;

figure(pltnum)
plot(tint,yv(:,1))
hold on 
plot(tint, wx_analytic,'--')
plot(tint,yv(:,2))
plot(tint, wy_analytic,'k:')
xlabel('t (sec)')
ylabel('w(t) rad/s')
grid on
legend('w_x(t) Numerical','w_x(t) Analytical','w_y(t) Numerical','w_y(t) Analytical','location','bestoutside')
% axis equal
pltnum = pltnum +1;

figure(pltnum)
plot(tint,yv(:,4))
hold on 
plot(tint, phi_x_analytic,'--')
plot(tint,yv(:,5))
plot(tint, phi_y_analytic,'k:')
xlabel('t (sec)')
ylabel('\phi(t) rad')
grid on
legend('\phi_x(t) Numerical','\phi_x(t) Analytical','\phi_y(t) Numerical','\phi_y(t) Analytical','location','bestoutside')
% axis equal
pltnum = pltnum +1;

figure(pltnum)
plot(tint,yv(:,3))
hold on 
plot(tint, wz_analytic,'--')
xlabel('t (sec)')
ylabel('w_z(t) rad/s')
grid on
legend('Numerical','Analytical')
% axis equal
pltnum = pltnum +1;

figure(pltnum)
plot(tint,yv(:,6))
hold on 
plot(tint, phi_z_analytic,'--')
xlabel('t (sec)')
ylabel('\phi_z(t) rad')
grid on
legend('Numerical','Analytical')
pltnum = pltnum +1;

figure(pltnum)
plot(Hx./Hz,Hy./Hz)
hold on
scatter(Hx(1)./Hz(1),Hy(1)./Hz(1),'g*')
scatter(Hx(end)./Hz(end),Hy(end)./Hz(end),'r*')
xlabel('Hx/Hz')
ylabel('Hy/Hz')
grid on
legend('Time history','@t=0','@t=7sec')
pltnum = pltnum +1;

%% Functions

%EOM
% Numerical Integration setup of Dynamics and Kinematics of attitude of a satellite
% Attitude is described by Euler Angles
function y = d(t,x, vars)
    
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

    if t < abs(mt0/mt_dot)
        lt = lt0 + lt_dot*t;
        mt = mt0 + mt_dot*t;
    else
        lt = lt0 + lt_dot*abs(mt0/mt_dot);
        mt = mt0 + mt_dot*abs(mt0/mt_dot);
    end

    h_noz_cm = ((l0+l_ic_prop)*mb + (l_ic_prop-lt)*mt)/(mb+mt);

    Mx = fz*(h_noz_cm*sind(alpha) + d_offset*cosd(alpha));
    
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
    
    % Dynamics
    dwx = (Mx + (Iy-Iz)*wy*wz - (Ixdot-mt_dot*(h_noz_cm^2 + d_offset^2))*wx)/Ix;
    dwy = ((Iz-Ix)*wz*wx - (Iydot - mt_dot*h_noz_cm^2)*wy)/Iy;
    dwz = ((Ix-Iy)*wx*wy - (Izdot - mt_dot*d_offset^2)*wz)/Iz;
    
    % 312 Euler Angle
    % Kinematics
    dpx = wx*cos(py)+wz*sin(py);
    dpy = wy-(wz*cos(py)-wx*sin(py))*tan(px);
    dpz = (wz*cos(py)-wx*sin(py))*sec(px);

    y = [dwx;dwy;dwz;dpx;dpy;dpz]; 
end

% Update PMOI based on changing satellite characterisitcs
function [Ix, Iy, Iz] = Ivals(t, vars)
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
    
    lt = zeros(length(t),1);
    mt = zeros(length(t),1);
    lt(1) = lt0;
    mt(1) = mt0;

    for i = 2:length(t)
        if t(i) < abs(mt0/mt_dot)
            lt(i) = lt(1) + lt_dot*t(i);
            mt(i) = mt(1) + mt_dot*t(i);
        else
            lt(i) = lt(1) + lt_dot*abs(mt0/mt_dot);
            mt(i) = mt(1) + mt_dot*abs(mt0/mt_dot);
        end
    end

    Itx = mt.*(r_motor^2/4 + lt.^2/3);
    Ity = mt.*(r_motor^2/4 + lt.^2/3);
    Itz = mt.*r_motor^2/2;
    
    Itxdot = mt_dot*(r_motor^2/4+lt.^2/3) + 2/3*mt.*lt.*lt_dot;
    Itydot = mt_dot*(r_motor^2/4+lt.^2/3) + 2/3*mt.*lt.*lt_dot;
    Itzdot = mt_dot*r_motor^2/2*ones(length(t),1);

    Ix = Ibx + Itx + mt.*(lt+l0).^2;
    Iy = Iby + Ity + mt.*(lt+l0).^2;
    Iz = Ibz + Itz;
end

% Analytical computation of angular velocity and euler angles
function [wx_analytic, wy_analytic,  phi_x_analytic, phi_y_analytic, phi_z_analytic] = analytic_vals(t, vars,yv)
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

    lt = zeros(length(t),1);
    mt = zeros(length(t),1);
    lt(1) = lt0;
    mt(1) = mt0;
 
    %Make Ix, Iy, Iz, Mx constant by dropping the secular terms
    for i = 2:length(t)
        if t(i) < abs(mt0/mt_dot)
            lt(i) = lt(1) + lt_dot*t(i)*0;
            mt(i) = mt(1) + mt_dot*t(i)*0;
        else
            lt(i) = lt(1) + lt_dot*abs(mt0/mt_dot)*0;
            mt(i) = mt(1) + mt_dot*abs(mt0/mt_dot)*0;
        end
    end
    
    h_noz_cm = ((l0+l_ic_prop)*mb + (l_ic_prop-lt).*mt)./(mb+mt);

    Mx = fz.*(h_noz_cm.*sind(alpha) + d_offset*cosd(alpha));
    a_val = -mt_dot.*h_noz_cm.^2;

    Itx = mt.*(r_motor^2/4 + lt.^2/3);
    Ity = mt.*(r_motor^2/4 + lt.^2/3);
    Itz = mt.*r_motor^2/2;
    
    Ix = Ibx + Itx + mt.*(lt+l0).^2;
    Iy = Iby + Ity + mt.*(lt+l0).^2;
    Iz = Ibz + Itz;
    
    kvect = (Iz-Ix)./Ix;
    wz0 = yv(1,3);
    
    wx_analytic = zeros(length(t),1);
    wy_analytic = zeros(length(t),1);
    phi_x_analytic = zeros(length(t),1);
    phi_y_analytic = zeros(length(t),1);
    phi_z_analytic = zeros(length(t),1);

    for i = 1:length(t)
       wx_analytic(i) =  a_val(i)*Mx(i)/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2) ...
        + exp(-a_val(i)*t(i)/Ix(i))*(-a_val(i)*Mx(i)*cos(kvect(i)*wz0*t(i)) ...
        + Ix(i)*kvect(i)*Mx(i)*wz0*sin(kvect(i)*wz0*t(i)))/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2);

       wy_analytic(i) = Ix(i)*kvect(i)*Mx(i)*wz0/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2) ...
           + exp(-a_val(i)*t(i)/Ix(i))*(-Ix(i)*kvect(i)*Mx(i)*wz0*cos(kvect(i)*wz0*t(i)) ...
           + -a_val(i)*Mx(i)*sin(kvect(i)*wz0*t(i)))/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2); 

       A0x = (a_val(i)^2 + Ix(i)^2*wz0^2*(1+kvect(i))^2)*(Ix(i)*kvect(i)*Mx(i)*wz0);
       A0y = (a_val(i)^2 + Ix(i)^2*wz0^2*(1+kvect(i))^2)*(a_val(i)*Mx(i));
       
       A1 = (a_val(i)^2 + Ix(i)^2*kvect(i)^2*wz0^2)*(-Ix(i)*(1+kvect(i))*Mx(i)*wz0);
       A2 = (a_val(i)^2 + Ix(i)^2*kvect(i)^2*wz0^2)*(a_val(i)*Mx(i));
       A3 = (a_val(i)^2 + Ix(i)^2*kvect(i)^2*wz0^2)*Mx(i) ...
           - (1+2*kvect(i))*Ix(i)*kvect(i)*Mx(i)*wz0*Ix(i)*wz0;
       A4 = (1+2*kvect(i))*(-Mx(i))*a_val(i)*Ix(i)*wz0;
       C = wz0*(a_val(i)^2 + Ix(i)^2*kvect(i)^2*wz0^2)*(a_val(i)^2 + Ix(i)^2*wz0^2*(1+kvect(i))^2);

       phi_x_analytic(i) = (A0x + A1*cos(wz0*t(i)) + A2*sin(wz0*t(i)) ...
           + Ix(i)*wz0*(A3*cos(kvect(i)*wz0*t(i)) + A4*sin(kvect(i)*wz0*t(i))) ...
            *(cosh(a_val(i)*t(i)/Ix(i))-sinh(a_val(i)*t(i)/Ix(i))))/C;

       phi_y_analytic(i) = (A0y - A1*sin(wz0*t(i)) + A2*cos(wz0*t(i)) ...
           + Ix(i)*wz0*(A3*sin(kvect(i)*wz0*t(i)) - A4*cos(kvect(i)*wz0*t(i))) ...
            *(cosh(a_val(i)*t(i)/Ix(i))-sinh(a_val(i)*t(i)/Ix(i))))/C;
    
       phi_z_analytic(i) = wz0*t(i);

    end
end