%------------------------------------------%
% Inverted Pendulum 
  
% "Kinematic and Dynamic Vehicle Models for Autonomous Driving
% Control Design", Borrelli, 2015

%------------------------------------------%


%% Dimensions

nx=5;  % No. of differential states
nu=2;  % No. of controls
nz=0;  % No. of algebraic states
ny=4; % No. of outputs
nyN=2; % No. of outputs at the terminal point
np=3; % No. of model parameters
nc=0; % No. of general constraints
ncN=0; % No. of general constraints at the terminal point
nbx = 2; % No. of bounds on states
nbu = 2; % No. of bounds on controls

% state and control bounds
nbx_idx = [4,5]; % indexs of states which are bounded
nbu_idx = [1,2]; % indexs of controls which are bounded

%% create variables

import casadi.*

states   = SX.sym('states',nx,1);   % differential states
controls = SX.sym('controls',nu,1); % control input
alg      = SX.sym('alg',nz,1);      % algebraic states
params   = SX.sym('params',np,1);    % parameters
refs     = SX.sym('refs',ny,1);     % references of the first N stages
refN     = SX.sym('refs',nyN,1);    % reference of the last stage
Q        = SX.sym('Q',ny,1);        % weighting matrix of the first N stages
QN       = SX.sym('QN',nyN,1);      % weighting matrix of the last stage
aux      = SX.sym('aux',ny,1);      % auxilary variable
auxN     = SX.sym('auxN',nyN,1);    % auxilary variable

%% Dynamics

k = params(1);
l_r = params(2);
l_f = params(3);

X = states(1); % global
Y = states(2); % global
psi = states(3);
e_y = states(4);
e_psi = states(5);

vx = controls(1); % local
d = controls(2);

% sideslip
beta = atan(l_r/(l_f+l_r)*tan(d));

% velocities
v = vx/cos(beta); % local
vy = v*sin(beta); % local

% global
dX = v*cos(psi+beta);
dY = v*sin(psi+beta);
dpsi = v/l_r*sin(beta);

% spatial reformulation
s_dot = (1/(1-k*e_y))*(vx*cos(e_psi)-vy*sin(e_psi));
dey = vx*sin(e_psi)+vy*cos(e_psi);
depsi = dpsi-k*s_dot;



% explicit ODE RHS
x_dot=[ dX; ...
        dY; ...
        dpsi; ... 
        dey; ...
        depsi ...
        ]./s_dot;  
 
% algebraic function
z_fun = [];                   

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;        
     
%% Objectives and constraints

% inner objectives
h = [e_y;e_psi;v;d];
hN = h(1:nyN);

% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);

% general inequality constraints
general_con = [];
general_con_N = [];

%% NMPC discretizing time length [s]

Ts_st = 0.01; % shooting interval time
