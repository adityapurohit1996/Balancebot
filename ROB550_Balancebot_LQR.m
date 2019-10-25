% Balancebot SS Model and LQR Control design
%The dynamics are second order derivative dynamics
%Theta is the body angle inclination and phi is the wheel rotation
clc
close all
clear 

%% constants
DT      = .01;          % 100hz controller loop
m_w     = .094;          % mass of one wheel in Kg MEASURED
m_b     = 1.1124;          % balancebot body mass without wheels (TO BE DETERMINED)%1282.4
R_w       = .042;          % radius of wheel in m MEASURED
L       = .0844;         % center of wheel to Center of mass (TO BE DETERMINED)
I_r     = 0.005869;        % Inertia of body about center (not wheel axis) Kg*m^2 (TO BE DETERMINED)
g       = 9.81;         % gravity m/s^2
R_gb    = 20.4;         % gearbox ratio
tau_s   = 1.8462;         % Motor output stall Torque @ V_nominal (TO BE DETERMINED)
w_free  = 46.42125;     % Motor output free run speed @ V_nominal (TO BE DETERMINED)
V_n     = 12.0;         % motor nominal drive voltage
I_gb = 3.1364*10^-5;     % inertial of motor armature and gearbox (TO BE DETERMINED)

% add inertia of wheels modeled as disks and times two for both sides
I_w = 2 * (I_gb+(m_w*R_w^2)/2);

%% statespace model including motor dynamics
a1 = I_w + (m_b + m_w)*R_w^2;
a2 = m_b * R_w * L;
a3 = I_r + m_b*L^2;
a4 = m_b * g * L;

% motor equation used: t = e*u - f*w
b1 = 2 * tau_s; % stall torque of two motors
b2  = b1 / w_free;   % constant provides zero torque @ free run

%simplifications
c1 = 1 - (a2^2)/(a1*a3);
c2 = 1 + (a2/a3);
c3 = 1 + (a2/a1);

%theta
%thetadot
%phi
%phidot

%% create statespace matricies
%working in cont time
A = [0 1 0 0;
     a4/(a3*c1) -(b2*c3)/(a3*c1) 0 (b2*c3)/(a3*c1);
     0 0 0 1;
     -(a2*a4)/(a1*a3*c1) (b2*c2)/(a1*c1) 0 -(b2*c2)/(a1*c1)];
B = [0 -(b1*c3)/(a3*c1) 0 (b1*c2)/(a1*c1) ]';
C = [1 0 0 0;
     0 0 1 0];
D = [0 0]';

states = {'theta' 'theta_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'theta'; 'phi'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_tf = tf(sys_ss)
% check poles and controllability of model
poles = eig(A)
controllability = rank(ctrb(sys_ss))

%              -171.1 s
%  ---------------------------------
%  s^3 + 13.17 s^2 - 94.44 s - 487.2

%%
%LQR Design
%Tune Your Gains Here and see the response
%Optimal gain that satifies our costs.
% The costs are for theta, thetadot, phi, phi dot
% to tune the gains 
%to tune down our duty cycle, we would up our R
%if theta angle is not right maybe up the cost 
% Q is a pos def symmetric matrix
% Because we are penalising the states independently Q is a diagonal
% matrix. 
% we are tuning the u gain and seeing how that affects the states
% note we have linearised the system about the vertical position
Q = [10 00 0 0
     0 0.01 0 0
     0 0 10 0
     0 0 0 0.08];
R = 10;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

% find Nbar for precompensator
% if we are interested in wheel position achieving 0 sserror. 
% it look what ref we need to provide in order to reach 0 sserror in the
% state we are interested in. 
% we are interested in the wheel position 
% how much do we need to scale that input for 0 sserror
% solves the linear system 
Cn = [0 0 1 0];
s = size(A,1);
Z = [zeros([1,s]) 1];
N = inv([A,B;Cn,0])*Z'; 
Nx = N(1:s);    % Nx and Nu say whata our states and input are at ss
Nu = N(1+s);
Nbar=Nu + K*Nx

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs)
sys_u = ss(Ac,Bc*Nbar,K,Nbar);

t = 0:0.01:4;
dist = 0.1; % move balancebot 10cm
angle = (dist/R_w)*ones(size(t));
angle(1) = 0;
[y,t,x]=lsim(sys_cl,angle,t); %lsim(sys_c1, angle, t, x0)   x0 = [0.1;0;0,0] statrts the sim from x0
[u,t,x]=lsim(sys_u,angle,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,R_w*y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','body angle (radians)')
set(get(AX(2),'Ylabel'),'String','body position (meters)')
title('Step Response with LQR Control and Precompensator')
figure
plot(t,u)

%% find discrete time system
%changing system to discrete
sys_d = c2d(sys_ss,DT,'zoh')
A_d = sys_d.a;
B_d = sys_d.b;
C_d = sys_d.c;
D_d = sys_d.d;
[K_d] = dlqr(A_d,B_d,Q,R)

%% discrete time closed loop
A_dc = [(A_d-B_d*K_d)];
B_dc = [B_d];
C_dc = [C_d];
D_dc = [D_d];

%pre-compenasator in discrete time
Nbar_d = -0.11 % adjust Nbar for steadystate performance


states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'phi'};
outputs = {'x'; 'phi'};
sys_dcl = ss(A_dc,B_dc*Nbar_d,C_dc,D_dc,DT,'statename',states,'inputname',inputs,'outputname',outputs);
sys_du = ss(A_dc,B_dc*Nbar_d,K_d,Nbar_d)
isstable(sys_dcl)

t = 0:0.01:4;
dist = 0.1; % move balancebot 10cm
angle = (dist/R_w)*ones(size(t));
[y,t,x]=lsim(sys_dcl,angle,t);
[u,t,x]=lsim(sys_du,angle,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,R_w*y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','body angle (radians)')
set(get(AX(2),'Ylabel'),'String','body position (meters)')
title('Discrete Step Response with LQR Control and Precompensator')
figure
plot(t,u)
