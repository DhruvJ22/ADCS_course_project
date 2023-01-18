%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dhruv Jain
% 23 November 2021 
% Purdue University
%
% Obj: Evaluation of Analytical Solution for angular velocties and body angles
%       Time varying: Mass of Propellant, Cener of Mass, Principal Moment of Inertia
%       Moto misalignment and offset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [wx_analytic, wy_analytic,  phi_x_analytic, phi_y_analytic, phi_z_analytic] = CubeSat_Analytical_Sol(t, vars,yv)
    
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

    lt = zeros(length(t),1);
    mt = zeros(length(t),1);
    lt(1) = lt0;
    mt(1) = mt0;
 
    %Make Ix, Iy, Iz, Mx constant by dropping the secular terms
    % Time varying Mass and Center of Mass
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
    
    % Motor misalignment
    Mx = fz.*(h_noz_cm.*sind(alpha) + d_offset*cosd(alpha));
    a_val = -mt_dot.*h_noz_cm.^2;
    
    % Time varying MOI
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
        % Compute angular velocities
        wx_analytic(i) =  a_val(i)*Mx(i)/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2) ...
        + exp(-a_val(i)*t(i)/Ix(i))*(-a_val(i)*Mx(i)*cos(kvect(i)*wz0*t(i)) ...
        + Ix(i)*kvect(i)*Mx(i)*wz0*sin(kvect(i)*wz0*t(i)))/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2);

       wy_analytic(i) = Ix(i)*kvect(i)*Mx(i)*wz0/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2) ...
           + exp(-a_val(i)*t(i)/Ix(i))*(-Ix(i)*kvect(i)*Mx(i)*wz0*cos(kvect(i)*wz0*t(i)) ...
           + -a_val(i)*Mx(i)*sin(kvect(i)*wz0*t(i)))/(a_val(i)^2+Ix(i)^2*kvect(i)^2*wz0^2); 
        
       % Compute rotation body angles
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