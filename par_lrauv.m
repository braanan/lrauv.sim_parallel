% par_lrauv.m Vehicle Simulator Testground
% Returns the time derivative of the state vector 
% Last modified July 17, 2014

function [ACCELERATIONS,FORCES] = par_lrauv(x,ui,xg,mqq,dcl)

% TERMS
% --------------------------------------------------------------------- 
% STATE VECTOR:
% x = [u v w p q r xpos ypos zpos phi theta psi]'
%  Body-referenced Coordinates
%  u            = Surge velocity            [m/sec]
%  v            = Sway velocity             [m/sec]
%  w            = Heave velocity            [m/sec]    
%  p            = Roll rate                 [rad/sec]
%  q            = Pitch rate                [rad/sec]
%  r            = Yaw rate                  [rad/sec]
%  Earth-fixed coordinates
%  xpos         = Position in x-direction   [m]
%  ypos         = Position in y-direction   [m]
%  zpos         = Position in z-direction   [m]
%  phi          = Roll angle                [rad]
%  theta        = Pitch angle               [rad]
%  psi          = Yaw angle                 [rad]
%
% INPUT VECTOR
% ui = [delta_s delta_r]'
%  Control Fin Angles
%  delta_s  = angle of stern planes         [rad]
%  delta_r  = angle of rudder planes        [rad]


% Optimization vars:
%--------------------------------------------------------------------- 
Xgp = xg;  % m         Vehicle center of gravity (x plane)
Mqq = mqq; % kg-m2*    Cross-flow drag (Mq|q|) 
dCL = dcl;     % n/a     Coef. of Lift Slope

% some vars to think of...
% xb =  0.0;          % m ***0.1181***
% Zww = -601.274653;  % kg/m      Cross-flow Drag

% Initialize global variables 
%--------------------------------------------------------------------- 
% load vdata          ;  % W and B, CG and CB coords
% par_vehicle_coeffs ;             % non-zero vehicle coefficients only

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
% xg =  0.0;          % m
yg =  0.0;          % m ***-0.000236***
zg =  0.0067940;    % m ***0.0067940***



% rB - vehicle centers of buoyancy
xb =  0.0;          % m ***0.1181***
yb =  0.0;          % m
zb =  0.0;          % m


Wp = W ;
Bp = B ;
% Xgp = xg;         % inputed to satisfy iteration!
Ygp = yg;
Zgp = zg;
Xbp = xb;
Ybp = yb;
Zbp = zb;

% Fins:
ARe     =   6.500000;     % n/a     Fin aspect ratios
% dCL     =   4.130000;     % n/a     Coef. of Lift Slope
CDc     =   0.030000;     % n/a     Crossflow Coef. of Drag !!try 0.6!!
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
% Mqq = -632.698957;  % kg-m2*    Cross-flow drag (Mq|q|)        *kg-m2/rad2?
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
%}


% Form the generalized mass matrix and invert it:
%--------------------------------------------------------------------- 
% [ ~, Minv ] = inv_massmatrix( 'vehicle_coeffs' );   % Minv matrix

M    = zeros(6,6);
M(1,:) = [ m-Xudot,          0,           0,         0,        m*Zgp,      -m*Ygp ];
M(2,:) = [       0,    m-Yvdot,           0,     -m*Zgp,           0, m*Xgp-Yrdot ];
M(3,:) = [       0,          0,     m-Zwdot,      m*Ygp, -m*Xgp-Zqdot,          0 ];
M(4,:) = [       0,      -m*Zgp,        m*Ygp, Ixx-Kpdot,           0,          0 ]; 
M(5,:) = [    m*Zgp,          0, -m*Xgp-Mwdot,         0,   Iyy-Mqdot,          0 ];
M(6,:) = [   -m*Ygp, m*Xgp-Nvdot,           0,         0,           0,  Izz-Nrdot ];

% M(:,1:6) = [m1; m2; m3; m4; m5; m6 ];
Minv = inv(M);
%}

% Output flags
% show_forces = 10 ;

% Get and check state variables and control inputs
%---------------------------------------------------------------------
% Get state variables
u   = x(1) ; v  = x(2) ; w  = x(3) ; p  = x(4) ; q  = x(5) ; r  = x(6);
phi = x(10) ; theta  = x(11) ; psi  = x(12) ;

% Get control inputs
% delta_s = ui(1) ; delta_r = ui(2) ; 
Xprop = ui(3) ; Xuu = ui(4) ; 

% Check control inputs (useful later)
% delta_max = 15*pi/180;
% if abs(delta_s) > delta_max
%     delta_s = sign(delta_s)*delta_max ;
% end;
% if abs(delta_r) > delta_max
%     delta_r = sign(delta_r)*delta_max ;
% end

% Initialize elements of coordinate system transform matrix
%---------------------------------------------------------------------
c1 = cos(phi); c2 = cos(theta); c3 = cos(psi); 
s1 = sin(phi); s2 = sin(theta); s3 = sin(psi); 
t2 = tan(theta);

% Get fin forces and moments
%---------------------------------------------------------------------
[ F1, F2, F3, F4, M1, M2, M3, M4 ] = parInline_robsFins( ui );

% [bodyLift, bodyMoment] = bodyLiftMoment(u, w);
% [ Yr, Zs, Ms, Nr ] = fins( rho, u, v, w, q, r, delta_r, delta_s );
% Set total forces from equations of motion
%---------------------------------------------------------------------


X = Xprop - (Wp-Bp)*s2 + Xuu*u*abs(u)...
    + m*(v*r - w*q + Xgp*(q*q + r*r) - Ygp*p*q - Zgp*p*r) ...
    + F1(1) + F2(1) + F3(1) + F4(1)...
    + Xvv*v*v + Xww*w*w + Xvr*v*r + Xwq*w*q + Xrr*r*r + Xqq*q*q;


Y = (Wp-Bp)*c2*s3 + Yvv*v*abs(v)...
    + Yuv*u*v + Yur*u*r + Yrr*r*abs(r) + Ywp*w*p...
    + m*(w*p - u*r + Ygp*(r*r+p*p) -Zgp*q*r - Xgp*p*q)...
    + F1(2) + F2(2) + F3(2) + F4(2);


Z = (Wp-Bp)*c2*c3 + Zww*w*abs(w) + Zqq*q*abs(q)...
    + Zuq*u*q + Zuw*u*w + Zvp*v*p...
    + m*(u*q - v*p + Zgp*(p*p + q*q) - Xgp*p*r - yg*q*r)...
    + F1(3) + F2(3) + F3(3) + F4(3) ; % + bodyLift ;


K = -(yg*W-yb*B)*cos(theta)*cos(phi) - (zg*W-zb*B)*cos(theta)*sin(phi) ...
    + Kpp*p*abs(p) - (Izz-Iyy)*q*r - (m*zg)*w*p + (m*zg)*u*r...
    + Kprop + M1(1) + M2(1) + M3(1) + M4(1);


M = -(Zgp*Wp - Zbp*Bp)*s2 - (Xgp*Wp - Xbp*Bp)*c2*c1...       % PITCH MOMENTS */
    + Mww*w*abs(w) + Mqq*q*abs(q) ...
    + Muw*u*w + Muq*u*q + Mpr*p*r...
    + (Izz - Ixx)*p*r...
    - m*(Zgp*(w*q - v*r) + Xgp*(u*q - v*p))...
    + M1(2) + M2(2) + M3(2) + M4(2) ; % + bodyMoment



N = (Ygp*Wp - Ybp*Bp)*s2 + (Xgp*Wp - Xbp*Bp)*c2*s1...         % YAW MOMENTS */
    + Nvv*v*abs(v) + Nrr*r*abs(r) + Nuv*u*v ...
    + Nur*u*r + Npq*p*q...
    + (Ixx - Iyy)*p*q...
    - m*Xgp*(u*r - w*p) + m*Ygp*(w*q - v*r)...
    + M1(3) + M2(3) + M3(3) + M4(3);

FORCES = [X Y Z K M N]' ;



ACCELERATIONS = ...
  [Minv(1,1)*X+Minv(1,2)*Y+Minv(1,3)*Z+Minv(1,4)*K+Minv(1,5)*M+Minv(1,6)*N
   Minv(2,1)*X+Minv(2,2)*Y+Minv(2,3)*Z+Minv(2,4)*K+Minv(2,5)*M+Minv(2,6)*N
   Minv(3,1)*X+Minv(3,2)*Y+Minv(3,3)*Z+Minv(3,4)*K+Minv(3,5)*M+Minv(3,6)*N
   Minv(4,1)*X+Minv(4,2)*Y+Minv(4,3)*Z+Minv(4,4)*K+Minv(4,5)*M+Minv(4,6)*N
   Minv(5,1)*X+Minv(5,2)*Y+Minv(5,3)*Z+Minv(5,4)*K+Minv(5,5)*M+Minv(5,6)*N
   Minv(6,1)*X+Minv(6,2)*Y+Minv(6,3)*Z+Minv(6,4)*K+Minv(6,5)*M+Minv(6,6)*N
   c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w
   s3*c2*u + (c1*c3+s1*s2*s3)*v + (c1*s2*s3-c3*s1)*w
     -s2*u +            c2*s1*v +            c1*c2*w 
         p +            s1*t2*q +            c1*t2*r
                           c1*q -               s1*r
                        s1/c2*q +            c1/c2*r] ;
                    

    
    function [ F1, F2, F3, F4, M1, M2, M3, M4 ] = parInline_robsFins( ui )
        
        % robsFins: Simulate lift/drag forces and moments applied to each of LRAUV
        %           fins.
        
        % Initialize global variables
        %---------------------------------------------------------------------
        % vehicle_coeffs ;
        
        % global Sfin ARe dCL CDc
        %{
        rho     =   1025;         % kg/m3
        ARe     =   6.500000;     % n/a     Fin aspect ratio
        dCL     =   1.5*4.130000;     % n/a     Coef. of Lift Slope
        CDc     =   0.600000;     % n/a     Crossflow Coef. of Drag !!try 0.6!!
        Cd0     =   0.030000;     % n/a     Min reaction drag coeff
        ec      =   0.9;
        Sfin    =   1.15e-2;      % m^2     Fin area
        % bfin  =   18.57e-2;     % m       Fin span
        % ARe   =   2*((bfin^2)/Sfin); % n/a   Effective aspect ratio
        %}
        stall_angle    =   30.000000*pi/180; % arcdeg
        
        % Position of fins from vehicle center:
        % colName = {'lowerRud','portElev','upperRud','stbdElev'};
        % rowName = {'X','Y','Z'};
        xi = 1 ; yi = 2 ; zi = 3 ; % dimention indecies
        finPos  = [ -0.633, -0.633, -0.633, -0.633 ;
                     0.012, -0.152,  0.012,  0.152 ;
                    -0.152,  0.000,  0.152,  0.000 ] ;  % m
        
        % Get and check state variables and control inputs
        %---------------------------------------------------------------------
        % Get state variables
        % u   = x(1) ; v  = x(2) ; w  = x(3) ; p  = x(4) ; q  = x(5) ; r  = x(6);
        
        % Get control inputs
        elev_ang = ui(1) ; rud_ang = ui(2) ;
        
        % Initialize elements of coordinate system transform matrix
        %---------------------------------------------------------------------
        fs1 = sin(rud_ang); fs2 = sin(elev_ang);
        fc1 = cos(rud_ang); fc2 = cos(elev_ang);
        
        
        % Calculate fin velocities in body coords
        %---------------------------------------------------------------------
        v1  = [ u + q*finPos(zi,1) - r*finPos(yi,1) ;
                v - p*finPos(zi,1) + r*finPos(xi,1) ;
                w + p*finPos(yi,1) - q*finPos(xi,1) ] ;
        
        v2  = [ u + q*finPos(zi,2) - r*finPos(yi,2) ;
                v - p*finPos(zi,2) + r*finPos(xi,2) ;
                w + p*finPos(yi,2) - q*finPos(xi,2) ] ;
        
        v3 	= [ u + q*finPos(zi,3) - r*finPos(yi,3) ;
                v + r*finPos(xi,3) - p*finPos(zi,3) ;
                w + p*finPos(yi,3) - q*finPos(xi,3) ] ;
        
        v4  = [ u + q*finPos(zi,4) - r*finPos(yi,4) ;
                v - p*finPos(zi,4) + r*finPos(xi,4) ;
                w + p*finPos(yi,4) - q*finPos(xi,4) ] ;
        
        
        % Now get angle of attack for each fin
        %---------------------------------------------------------------------
        norm_v1 = sqrt( v1(1)*v1(1) + v1(2)*v1(2) + v1(3)*v1(3) );
        if (norm_v1 < 0.001)
            alpha1 = 0.0;
        else
            alpha1 =  rud_ang - v1(2)/v1(1); % asin(-v1(2)/norm_v1) + rud_ang;
        end
        
        norm_v2 = sqrt( v2(1)*v2(1) + v2(2)*v2(2) + v2(3)*v2(3) );
        if (norm_v2 < 0.001)
            alpha2 = 0.0;
        else
            alpha2 =  elev_ang + v2(3)/v2(1); % atan2(v2(3),v2(1)) + elev_ang;
        end
        
        norm_v3 = sqrt( v3(1)*v3(1) + v3(2)*v3(2) + v3(3)*v3(3) );
        if (norm_v3 < 0.001)
            alpha3=0.0;
        else
            alpha3 = rud_ang - v3(2)/v3(1); %asin(-v3(2)/norm_v3) + rud_ang; %
        end
        
        norm_v4 = sqrt( v4(1)*v4(1) + v4(2)*v4(2) + v4(3)*v4(3) );
        if (norm_v4 < 0.001)
            alpha4=0.0;
        else
            alpha4 = elev_ang + v4(3)/v4(1); % atan2(v4(3),v4(1)) + elev_ang;
        end
        
        % lift and drag coefficients */
        CDC = CDc/ARe;
        
        % Note that if stall angle is exceeded: NO LIFT */
        if (abs(alpha1) < stall_angle)
            CL1 = dCL*alpha1 + CDC*alpha1*abs(alpha1);
        else CL1 = 0. ;
        end
        
        if (abs(alpha2) < stall_angle)
            CL2 = dCL*alpha2 + CDC*alpha2*abs(alpha2);
        else CL2 = 0. ;
        end
        
        if (abs(alpha3) < stall_angle)
            CL3 = dCL*alpha3 + CDC*alpha3*abs(alpha3);
        else CL3 = 0. ;
        end
        
        if (abs(alpha4) < stall_angle)
            CL4 = dCL*alpha4 + CDC*alpha4*abs(alpha4);
        else CL4 = 0. ;
        end
        
        aa = 1.0/(pi*ARe*ec);
        
        CD1 = Cd0 + aa*CL1*CL1;
        CD2 = Cd0 + aa*CL2*CL2;
        CD3 = Cd0 + aa*CL3*CL3;
        CD4 = Cd0 + aa*CL4*CL4;
        
        % lift and drag forces, in flow coords... */
        %---------------------------------------------------------------------
        cons = (rho*Sfin)/2.0;
        
        LW1 = cons*norm_v1*norm_v1*CL1;      % positive when the lift vector is close to normal vector */
        LW2 = cons*norm_v2*norm_v2*CL2;
        LW3 = cons*norm_v3*norm_v3*CL3;
        LW4 = cons*norm_v4*norm_v4*CL4;
        
        DW1 = cons*norm_v1*norm_v1*CD1;      % always positive */
        DW2 = cons*norm_v2*norm_v2*CD2;
        DW3 = cons*norm_v3*norm_v3*CD3;
        DW4 = cons*norm_v4*norm_v4*CD4;
        
        LF1 = LW1*cos(alpha1) + DW1*sin(alpha1);        % force in the fin normal direction */
        LF2 = LW2*cos(alpha2) + DW2*sin(alpha2);
        LF3 = LW3*cos(alpha3) + DW3*sin(alpha3);
        LF4 = LW4*cos(alpha4) + DW4*sin(alpha4);
        
        DF1 = -LW1*sin(alpha1) + DW1*cos(alpha1);       % force in the fin-aft direction */
        DF2 = -LW2*sin(alpha2) + DW2*cos(alpha2);
        DF3 = -LW3*sin(alpha3) + DW3*cos(alpha3);
        DF4 = -LW4*sin(alpha4) + DW4*cos(alpha4);
        
        % Finally, transform into the body frame */
        %---------------------------------------------------------------------
        F1  = [ -LF1*fs1 + (-DF1)*fc1 ;
                 LF1*fc1 + (-DF1)*fs1 ;
                 0.0                  ] ;
        
        F2  = [ -LF2*fs2 + (-DF2)*fc2 ;
                 0.0                  ;
                -LF2*fc2 - (-DF2)*fs2 ] ;
        
        F3	= [ -LF3*fs1 + (-DF3)*fc1 ;
                 LF3*fc1 + (-DF3)*fs1 ;
                 0.0                  ] ;
        
        F4  = [ -LF4*fs2 + (-DF4)*fc2 ;
                 0.0                  ;
                -LF4*fc2 - (-DF4)*fs2 ] ;
        
        % moments induced by these forces */
        M1  = [ finPos(yi,1)*F1(3) - finPos(zi,1)*F1(2) ;
               -finPos(xi,1)*F1(3) + finPos(zi,1)*F1(1) ;
                finPos(xi,1)*F1(2) - finPos(yi,1)*F1(1) ] ;
        
        M2  = [ finPos(yi,2)*F2(3) - finPos(zi,2)*F2(2) ;
               -finPos(xi,2)*F2(3) + finPos(zi,2)*F2(1) ;
                finPos(xi,2)*F2(2) - finPos(yi,2)*F2(1) ] ;
        
        M3  = [ finPos(yi,3)*F3(3) - finPos(zi,3)*F3(2) ;
               -finPos(xi,3)*F3(3) + finPos(zi,3)*F3(1) ;
                finPos(xi,3)*F3(2) - finPos(yi,3)*F3(1) ] ;
        
        M4  = [ finPos(yi,4)*F4(3) - finPos(zi,4)*F4(2) ;
               -finPos(xi,4)*F4(3) + finPos(zi,4)*F4(1) ;
                finPos(xi,4)*F4(2) - finPos(yi,4)*F4(1) ] ;
    end


end