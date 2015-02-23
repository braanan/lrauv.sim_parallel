% par_minimizeError_LRAUV_SIM.M    -   For LRAUV Vehicle Simulator

% ele_offsetLRAUV_SIM : Minimizes error intruduced by off-set in elevator
%                       angle data.
%
% Last modified July 21, 2014
% Ben Raanan


clear
close all


% h = waitbar(0,'Initializing error minimization...');
fpath = '~/Documents/MATLAB/MBARI/mat/workver/'; % shark/

% filename = [fpath 'LRAUV_SIM_201309282307_201309301141.mat']; % shark data
filename = [fpath 'LRAUV_SIM_201309121813_201309140344.mat'];  % bottoming



% INITIALIZE OPTIMIZATION
%--------------------------------------------------------------------------
startTime = datestr(clock)

% STATE AND INPUT VECTORS:
% x = [u v w p q r xpos ypos zpos phi theta psi]'
% ui = [ delta_s delta_r Xprop Kprop ]'
[ time, time_step, xstruct, names, controls ] = initialize_LRAUV_SIM( filename );


timeIn  = datenum(2013,09,12,20,21,12);
timeOut = datenum(2013,09,30,14,28,24);
timeEval    = 240; % sec, evaluation run time
timeIni = closest(timeIn,time); 

tryVal1 = -0.3:0.05:-0.1;
tryVal2 = (0.1:0.1:1)*-632.698957; % linspace(-1,-0.8,8);

% Define error range and resolution
ele_offset  = -0.9; % deg 
pLash       = 0; % 4;   % deg 
nLash       = 0;   % deg 

rud_offset  = 0;    % deg 

% Define "global" vars:
Mqq     =   tryVal2;% 0.35*-632.698957;   % kg-m2     Cross-flow drag (Mq|q|) 
dCL     =   1.5*4.130000;       % n/a     Coef. of Lift Slope
%{
% ARe     =   6.500000;           % n/a     Fin aspect ratio
% CDc     =   0.030000;           % n/a     Crossflow Coef. of Drag !!try 0.6!!
% zg      =   0.0067940;          % m         Center of gravity             
% Sfin    =   1.15e-2;            % m^2       Fin area
%}



% initiate first step and set runtime
%--------------------------------------------------------------------------
startPoint = timeIni;
n_steps     = fix(timeEval/time_step); %size(ui,1); 
n = startPoint:startPoint+n_steps;

% Define global vars
%{
%global  Sfin ARe dCL CDc % xg zg Mqq 
% zg      =   0.0067940;          % m         Center of gravity             
% Sfin    =   1.15e-2;            % m^2       Fin area
% Mqq     =   0.35*-632.698957;   % kg-m2     Cross-flow drag (Mq|q|) 
% ARe     =   6.500000;           % n/a     Fin aspect ratio
% dCL     =   1.5*4.130000;       % n/a     Coef. of Lift Slope
% CDc     =   0.030000;           % n/a     Crossflow Coef. of Drag !!try 0.6!!
%}


% Set up mass-shifting
%--------------------------------------------------------------------------
mass        =   147.8671;         % kg Flooded Vehicle total mass
movableMass =   26;               % kg Battary movable mass
dropWtMass  =   1.0;              % kg Mass of the drop weight #1, kg
dropWtX     =  -0.1330;           % m  X location of the drop weight #1, m
Xmass = (movableMass.*xstruct.mass_p + dropWtMass*dropWtX)./mass;


% Optimization utill
%--------------------------------------------------------------------------
cloop=1; 
nVal1=length(tryVal1); %*length(tryVal2);
nVal2=length(tryVal2); %*length(tryVal2);
error=zeros(length(tryVal2),length(tryVal1));

% waitbar(0.01,h,'Runing error minimization loops...');


% RUN OPTIMIZATION LOOPS
%--------------------------------------------------------------------------
tic
parfor b = 1:nVal1
   
    
    dropWtX = tryVal1(b);
    Xmass   = (movableMass.*xstruct.mass_p(n) + dropWtMass*dropWtX)./mass;

    % mqq = Mqq;      % Cross-flow drag (Mq|q|)
    dcl = dCL;      % Coef. of Lift Slope
    
    for k = 1:nVal2
        
        mqq = Mqq(k);      % Cross-flow drag (Mq|q|)
        
        % Reset control vars
        ui = controls(n,:);
        
        % Correct for offsets in data:
        ui(:,1) = ui(:,1) + ele_offset*pi/180; % ele_offset
        ui(:,2) = ui(:,2) + rud_offset*pi/180;
        
        % Correct for hysteresis and backlash offsets
        %{
        ui(ui(:,1)>pLash*180/pi,1) = ui(ui(:,1)>pLash*180/pi,1) + 1*pi/180;
        ui(ui(:,1)<nLash*180/pi,1) = ui(ui(:,1)<nLash*180/pi,1) + -0.65*pi/180;
        %}
        
        % Unpack state vector
        x = zeros(1,12);
        for c=[1:6,9,10:12];
            x(c) = xstruct.(names{c})(startPoint);
        end; 
        
        
        simlog = zeros(n_steps,16);
        % Run simulation
        for i = 1:n_steps
            
            % Account for movable mass shift
            % xg = Xmass(i);
            
            % Set some vars constant
            x(1) = xstruct.u(n(i));
            x(4)  = xstruct.p(n(i));
            x(10) = xstruct.phi(n(i));
            
            ui_in = ui(i,:);
            
            % Calc next step
            [xdot,~] = par_lrauv(x,ui_in,Xmass(i),mqq,dcl); 
            
            % Log step data
            simlog(i,:) = [x ui_in];
            
            % RUNGE-KUTTA APPROXIMATION to calculate new states
            % NOTE: ideally, should be approximating ui values for k2,k3
            k1_vec = xdot;
            k2_vec = par_lrauv(x+(0.5.*time_step.*k1_vec)', ((ui(i,:)+ui(i+1,:))./2), (Xmass(i)+Xmass(i+1))./2,mqq,dcl) ;
            k3_vec = par_lrauv(x+(0.5.*time_step.*k2_vec)', ((ui(i,:)+ui(i+1,:))./2), (Xmass(i)+Xmass(i+1))./2,mqq,dcl) ;
            k4_vec = par_lrauv(x+(time_step.*k3_vec)', ui(i+1,:), Xmass(i+1),mqq,dcl) ;
            x = x + time_step/6.*(k1_vec +2.*k2_vec +2.*k3_vec +k4_vec)';
            
        end; 
        
        
        % Compute squered sum of residuales and find minimizing offset value
        theta_m = simlog(:,11);
        % Quadratic loss function
        res     = (theta_m - xstruct.theta(n(1):n(end-1))').^2;
        error(k,b) = sum(res);
        
        % update waitbar
        %{
        cloop=cloop+1;
        prg=cloop/nTry;
        waitbar(prg,h,['Runing error minimization loop ' num2str(b) ' ['...
            num2str(100*prg,2) '%]'] );
        %}
        
    end
    
    display([ datestr(clock) ': Iteration cycle ' num2str(b) ' of ' num2str(nVal1) ' complete.']);
end; toc
% waitbar(1,h,['Error minimization complete [' num2str(100) '%]'] );

% plot results
figure;
set(gcf,'Units','normalized','Position',[0.1 0.2 0.8 0.8],...
    'PaperPositionMode','auto');
surf(tryVal1,tryVal2,error)
title('LRAUV simulator error minimization function',...
    'fontweight','bold','fontsize',22)

% set(gca,'xtickLabel',get(gca,'xtick')*-632.698957);
% set(gca,'ytickLabel',get(gca,'ytick')*4.13);
%{
xlabel('Deviation from static X center of gravity (cm)',...
    'fontweight','bold','fontsize',16)
set(gca,'xticklabel',100*dropWtMass*tryVal1/mass)
set(get(gca,'xlabel'),'rotation',-1);
ylabel('Elevator offset correction (deg)',...
    'fontweight','bold','fontsize',16)
set(get(gca,'ylabel'),'rotation',74);
zlabel('\Sigma Error',...
    'fontweight','bold','fontsize',16)
%}
box off; grid on;
%}

minError = min(min(error));
[row,col] = find(error==minError);

optVals = [minError,tryVal1(col),tryVal2(row)] 


%{
figure;
plot(xstruct.theta(n)*(180/pi),'ro');
hold on;
plot(theta_m' * (180/pi));
%}