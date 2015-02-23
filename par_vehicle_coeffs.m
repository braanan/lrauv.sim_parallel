% par_vehicle_coeffs.m
% Nov 11, 2014


% Mass Properties:
rho         = 1025;              % kg/m3   
g           = 9.80665;           % m/s2
mass        = 147.8671;          % kg Flooded Vehicle mass

% mass        = .110*rho;
volume      = 0.144260585365854; % m3 (equals 1450 N buoyancy in 1025 kg/m3 water)

% excludes buoyancy bladder at default settings
m = mass;           % kg, mass
W = m*g ;           % N, Weight                         % B = mass*g; 
B = rho*volume*g;   % N, Buoyancy

% Geometric Parameters (Used only by the simulation):
% rG - vehicle centers of gravity
xg =  0.0;          % m
yg =  0.0;          % m ***-0.000236***
zg =  0.0067940;    % m ***0.0067940***



% rB - vehicle centers of buoyancy
xb =  0.0;          % m ***0.1181***
yb =  0.0;          % m
zb =  0.0;          % m


Wp = W ;
Bp = B ;
Xgp = xg;         % called within par_lrauv.m to stisfy iteration!
Ygp = yg;
Zgp = zg;
Xbp = xb;
Ybp = yb;
Zbp = zb;

% Fins:
ARe     =   6.500000;     % n/a     Fin aspect ratios
dCL     =   4.130000;     % n/a     Coef. of Lift Slope
CDc     =   0.600000;     % n/a     Crossflow Coef. of Drag !!try 0.6!!
Cd0     =   0.030000;     % n/a     Min reaction drag coeff
ec      =   0.9;
Sfin    =   1.15e-2;      % m^2     Fin area

% Mass Properties:
Ixx =  3.000000;    % kg-m2     Diagonal inertia tensor
Iyy =  41.980233;   % kg-m2     Diagonal inertia tensor
Izz =  41.980233;   % kg-m2     Diagonal inertia tensor

% Thruster parameters:
Kpp   = -0.191601;    % kg-m2*  Rolling Resistance             *kg-m2/rad2?
Kprop =  0.23;        % N-m     Propeller Torque ***0.23***

% Added Mass:
Yvdot  = -126.324739; % kg;     // Yvdot, kg.
Zwdot  = -126.324739; % kg;     // Zwdot, kg.
Xudot  =   -4.876161; % kg;     // Xudot, kg.
Mqdot  =  -33.463086; % kg-m2;  // Mqdot, kg-m^2.
Nrdot  =  -33.463086; % kg-m2;  // Nrdot, kg-m^2.
Kpdot  =    0.000000; % kg-m2;  // Kpdot, kg-m^2.
Kvdot  =    0.000000; % kg-m;   // Kvdot, kg-m.
Mwdot  =    7.117842; % kg-m;   // Mwdot, kg-m.
Zqdot  =    7.117842; % kg-m;   // Zqdot, kg-m.
Nvdot  =   -7.117842; % kg-m;   // Nvdot, kg-m.
Yrdot  =   -7.117842; % kg-m;   // Yrdot, kg-m.
Ypdot  =    0.000000; % kg-m;   // Ypdot, kg-m.

% Stability Derivatives:
Xqq =  7.117842;    % kg-m;
Xrr =  7.117842;    % kg-m;
Xvv = -54.370919;   % kg/m;   // Xvv   , kg/m
Xww = -54.370919;   % kg/m;   // Xww   , kg/m
Yvv = -601.274653;  % kg/m      Cross-flow Drag (Yv|v|)
Yrr =  0.000000;    % n/a*      Cross-flow Drag (Yr|r|)         *kg-m/rad2?
Zww = -601.274653;  % kg/m      Cross-flow Drag
Zqq =  0.000000;    % n/a*      Cross-flow Drag (Zq|q|)
Mww = -58.113144;   % kg        Cross-flow drag (-Nv|v|)
Mqq = -632.698957;  % kg-m2*    Cross-flow drag (Mq|q|)        *kg-m2/rad2?
Nvv =  58.113144;   % kg     	Cross-flow drag (Nv|v|)
Nrr = -632.698957;  % kg-m2* 	Cross-flow drag (Nr|r|)        *kg-m2/rad2?

Yuv = -23.954759;   % kg/m      Body Lift Force and Fin Lift
Zuw = -23.954759;   % kg/m      Body Lift Force and Fin Lift
Nuv = -105.660262;  % kg        Body and Fin Lift and Munk Moment
Muw =  105.660262;  % kg        Body and Fin Lift and Munk Moment

Xwq = -126.324739;  % kg;
Xvr =  126.324739;  % kg;
Yur =  8.719853;    % kg*       Added Mass Cross-term and Fin Lift *kg/rad?
Zuq = -8.719853;    % kg*       Added Mass Cross-term and Fin Lift *kg/rad?
Nur = -61.182063;   % kg-m*     Added Mass Cross-term and Fin Lift *kg-m/rad?
Muq = -61.182063;   % kg-m*     Added Mass Cross-term and Fin Lift *kg-m/rad?

Ypq = -7.117842;    % kg-m      Added Mass Cross-term (-Zqdot)
Ywp =  126.324739;  % kg-m*     Added Mass Cross-term              *kg/rad?
Zvp = -126.324739;  % kg*       Added Mass Cross-term              *kg/rad?
Zrp = -7.117842;    % kg-m*     Added Mass Cross-term (Yrdot)      *kg/rad?
Mpr =  33.463086;   % kg-m2;  // Mpr   , kg-m^2
Mrp =  33.463086;   % kg-m2*    Added Mass Cross-term          *kg-m2/rad2?
Mvp =  7.117842;    % kg-m*     Added Mass Cross-term (-Yrdot)   *kg-m/rad?
Npq = -33.463086;   % kg-m2*    Added Mass Cross-term          *kg-m2/rad2?
Nwp =  7.117842;    % kg-m*   	Added Mass Cross-term (Zqdot)    *kg-m/rad?
