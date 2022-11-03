%% path

addpath(genpath([pwd,'/nmpc']));

OS_MAC = 0;
OS_LINUX = 0;
OS_WIN = 0;

if ismac
    OS_MAC = 1;
    rmpath(genpath([pwd,'/nmpc/solver/linux']));
    rmpath(genpath([pwd,'/nmpc/solver/windows']));
elseif isunix
    OS_LINUX = 1;
    rmpath(genpath([pwd,'/nmpc/solver/mac']));
    rmpath(genpath([pwd,'/nmpc/solver/windows']));
elseif ispc
    OS_WIN = 1;
    rmpath(genpath([pwd,'/nmpc/solver/mac']));
    rmpath(genpath([pwd,'/nmpc/solver/linux']));
else
    disp('Platform not supported')
end



%% settings

if exist([pwd,'/nmpc_sim/nmpc/data/settings.mat'],'file')==2
    load nmpc/data/settings.mat
else 
    error('No setting data is detected!');
end

% user defined
settings.N = 80;
settings.N2 = settings.N/5;
settings.r = 10;

%% opt

opt.hessian         = 'Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator      = 'ERK4'; % 'ERK4','IRK3','IRK3-DAE'
opt.condensing      = 'no';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver        = 'hpipm_sparse'; 
opt.hotstart        = 'no'; %'yes','no' (only for qpoases, use 'no' for nonlinear systems)
opt.shifting        = 'no'; % 'yes','no'
opt.nonuniform_grid = 0; % if use non-uniform grid discretization (go to InitMemory.m, line 459 to configure)
opt.RTI             = 'yes'; % if use Real-time Iteration

% available qpsolvers
%'qpoases' (condensing is needed)
%'qpoases_mb' (move blocking strategy)
%'quadprog_dense' (for full condensing)
%'hpipm_sparse' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'hpipm_pcond' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'ipopt_dense' (install OPTI Toolbox first; for full condensing)
%'ipopt_sparse' (install OPTI Toolbox first; set opt.condensing='no')
%'ipopt_partial_sparse'(set opt.condensing='partial_condensing'; only for state and control bounded problems)
%'osqp_sparse' (set opt.condensing='no')
%'osqp_partial_sparse' (set opt.condensing='partial_condensing')
%'qpalm_cond' (condensing is needed)
%'qpalm_sparse'(set opt.condensing='no')

%% input

if opt.nonuniform_grid
    [input, data] = InitData_ngrid(settings);
    settings.N = settings.r;
else
	[input] = InitData(settings);
end  

%% mem

mem = InitMemory(settings, opt, input);

%% track

config.ref_div = 100;
track = mpc_create_reference('sparcs_track_small.drd', settings.Ts_st/config.ref_div, settings.N+1);
track.v = 0.75*ones(length(track.s));%smooth(1./(abs(track.k)+1),10/settings.Ts_st);

% R = 2;
% vel = 1.5;
% config.ref_div = 100;
% track = mpc_create_circle(R,vel,settings.Ts_st/config.ref_div, settings.N+1);

% path_length = 100;
% vel = 1;
% track = mpc_create_line(path_length,vel,settings.Ts_st, settings.N+1);

%% sim

mem.iter = 1; time = 0.0;
Tf = track.s(end-(settings.N+1));  % simulation space
state_sim = input.x0';
controls_MPC = input.u0';
y_sim = [];
constraints = [];
stats.CPT = [];
stats.KKT = [];
stats.OBJ = [];
stats.numIT=[];
computed_errors = [];

last_ref_index = -1;

%% config

config.debug = 1;
