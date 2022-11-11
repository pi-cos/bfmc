function [ctrls] = nmpc_controller(states)
%NMPC_CONTROLLER Computes the controls for the RC car
%   Inputs:
%        - states: contains car position and velocities from localisation
%   Output:
%        - ctrls: contains steering and velocity inputs fro RC car

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File Name: nmpc_controller.m                                            %
%                                                                         %
% Designed By: Enrico Picotti                                             %
% Company    : UniPD                                                      %
% Project    : Bosch RC Car                                               %
% Purpose    : Implements the NMPC controller                             %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% init

% needed variables
persistent iter last_ref_index 
persistent track settings opt mem input config
persistent init_X init_Y init_PSI

% debug variables
persistent controls_MPC state_sim y_sim constraints stats computed_errors state_cosim

% needed variables
if isempty(iter)
    iter = 1;
    last_ref_index = evalin('base','last_ref_index');
    % static variables
    track       = evalin('base','track');
    settings    = evalin('base','settings');
    config      = evalin('base','config');
    opt         = evalin('base','opt');
    % updated at each iteration variables
    mem         = evalin('base','mem');
    input       = evalin('base','input');
end

% debug variables
if (config.debug && isempty(state_sim))
    state_sim       = evalin('base','state_sim');
    controls_MPC    = evalin('base','controls_MPC');
    y_sim           = evalin('base','y_sim');
    constraints     = evalin('base','constraints');
    stats           = evalin('base','stats');
    computed_errors = evalin('base','computed_errors');
    state_cosim = [];
end

%% input set

newLoc    = states(1);
X_real    = states(2);
Y_real    = states(3);
PSI_real  = states(4);
VX        = states(5);
DF        = states(6);

w1 = states(7);
w2 = states(8);
w3 = states(9);
w4 = states(10);
w5 = states(11);

%% init pos and rot 

if newLoc < 0.5
    reset = 0;
    stop = 0;
    ctrls = [zeros(settings.nu,1);0;stop;reset;0;0;0;0];
    return
end

if iter < 10
    reset = 1;
    stop = 0;
    ctrls = [zeros(settings.nu,1);0;stop;reset;0;0;0;0];
    iter = iter+1;
    return
end

if iter == 10
    init_X = X_real;
    init_Y = Y_real;
    init_PSI = PSI_real;

    Q=repmat([w1 w2 w3 w4 w5]',1,settings.N);
    QN=Q(1:settings.nyN,1);

    input.W=Q;           % (ny by N)
    input.WN=QN;         % (nyN by 1)

end

if VX < 0.1
    VX = 0.1;
    warning('Settings VX to 0.1!')
end

PSI = PSI_real - init_PSI;
rotMat = @(psi) [cos(psi) -sin(psi); sin(psi), cos(psi)];
XY = rotMat(-init_PSI)*[X_real - init_X; Y_real - init_Y];
X = XY(1);
Y = XY(2);
state_cosim = [state_cosim,[XY;PSI;VX;DF]];

%% find current pos on reference

search_indexes = (1:length(track.s)-(settings.N+1));
squared_dist = (track.X(search_indexes)-X).^2 + (track.Y(search_indexes)-Y).^2;
[~, curr_ref_index] = min(squared_dist);
last_ref_index = curr_ref_index;

if track.s(curr_ref_index) > track.s(end-(settings.N+1)-0.5*config.ref_div/settings.Ts_st) && iter > 100
    disp(' ------------------------------ ')
    disp(' ****** TRACK COMPLETED! ****** ')
    reset = 1;
    stop = 1;
    ctrls = [zeros(settings.nu,1);0;stop;reset;0;0;0;1];
    return
else
    stop = 0;
end

%% compute current errors wrt track centerline
current_state.s = track.s(curr_ref_index);%time(end);
current_state.X = X;
current_state.Y = Y;
current_state.psi = PSI;
[e_y, e_psi] = errors_calculator(current_state, curr_ref_index, track);

e_v = track.v(curr_ref_index)-VX;

computed_errors(mem.iter,:) = [e_y, e_psi, e_v];

%% update input for NMPC
% update state
input.x0 = [X;Y;PSI;e_y;e_psi;VX;DF];

% update ref
% the reference input.y is a ny by N matrix
% the reference input.yN is a nyN by 1 vector
input.y = zeros(settings.ny,settings.N);
input.yN = zeros(settings.nyN,1);
% curvature
ref_samples = curr_ref_index:config.ref_div:curr_ref_index+settings.N*config.ref_div;
ref_samples_mod = mod(ref_samples,length(track.s)-(settings.N+1));
if any(ref_samples ~= ref_samples_mod)
    idx_zero = find(ref_samples_mod==0);
    ref_samples_mod(idx_zero:end)=ref_samples_mod(idx_zero:end)+1;
    ref_samples = ref_samples_mod;
end
input.y(3,:) = track.v(ref_samples(1:end-1));
input.yN(3) = track.v(ref_samples(end));

% update params
input.od(1,:) = track.k(ref_samples);

%% NMPC

if any(input.x(6,:) < 0.1)
    input.x(6,input.x(6,:)<0.25) = 0.25;
    warning('Settings input.x to 0.25!')
end

% call the NMPC solver 
try
    [output, mem] = mpc_nmpcsolver(input, settings, mem, opt);
catch
%     output = input;
%     output.info.cpuTime = 0;
%     output.info.shootTime = 0;
%     output.info.condTime = 0;
%     output.info.qpTime = 0;
%     output.info.OptCrit = 0;
%     output.info.objValue = 0;
%     output.info.iteration_num = 0;
    warning('failed NMPC.');
    reset = 1;
    stop = 1;
    ctrls = [zeros(settings.nu,1);0;stop;reset;0;0;0;0];
    return
end

% obtain the solution and update the data
    switch opt.shifting
        case 'yes'
            input.x=[output.x(:,2:end),output.x(:,end)];  
            input.u=[output.u(:,2:end),output.u(:,end)];
            input.z=[output.z(:,2:end),output.z(:,end)];
            input.lambda=[output.lambda(:,2:end),output.lambda(:,end)];
            input.mu=[output.mu(settings.nc+1:end);output.mu(end-settings.nc+1:end)];
            input.mu_x=[output.mu_x(settings.nbx+1:end);output.mu_x(end-settings.nbx+1:end)];
            input.mu_u=[output.mu_u(settings.nu+1:end);output.mu_u(end-settings.nu+1:end)];
            
        case 'no'
            input.x=output.x;
            input.u=output.u;
            input.z=output.z;
            input.lambda=output.lambda;
            input.mu=output.mu;
            input.mu_x=output.mu_x;
            input.mu_u=output.mu_u;
    end

% collect the statistics
cpt=output.info.cpuTime;
tshooting=output.info.shootTime;
tcond=output.info.condTime;
tqp=output.info.qpTime;
OptCrit=output.info.OptCrit;

if config.debug
    
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(:,1);
    sim_input.z = input.z(:,1);
    sim_input.p = input.od(:,1);

    % simulate system
    [xf, zf] = Simulate_System(sim_input.x, sim_input.u, sim_input.z, sim_input.p, mem, settings);
    xf = full(xf);

    % Collect outputs
    y_sim = [y_sim; full(h_fun('h_fun', xf, sim_input.u, sim_input.p))'];

    % Collect constraints
    constraints=[constraints; full( path_con_fun('path_con_fun', xf, sim_input.u, sim_input.p) )'];

    % store the optimal solution and states
    controls_MPC = [controls_MPC; output.u(:,1)'];
    state_sim = [state_sim; xf'];
    stats.KKT= [stats.KKT;OptCrit];
    stats.OBJ= [stats.OBJ;output.info.objValue];
    stats.CPT = [stats.CPT; cpt, tshooting, tcond, tqp];
    stats.numIT = [stats.numIT; output.info.iteration_num];
end

% go to the next sampling instant
% nextTime = mem.iter*settings.Ts_st;
mem.iter = mem.iter+1;
% if mod(mem.iter,10)==1
    disp(['Iter ',num2str(mem.iter),', current space:' num2str(current_state.s) 'm, X =', num2str(X),'m, Y = ',num2str(Y),'m, e_y = ' num2str(e_y), ', CPT:' num2str(cpt) 'ms Opt:' num2str(OptCrit) '   OBJ:' num2str(output.info.objValue) '  SQP_IT:' num2str(output.info.iteration_num)]);
% end
iter = iter+1;

%% output
reset = 0;
ctrls = [output.u(1,1);output.u(2,1);track.k(ref_samples(1));stop;reset;e_y;e_psi;e_v;0];

%% clean me up function, called when stop

% assignin('base','mem',mem);

%     function cleanMeUp()
        if config.debug 
%             computed_errors(mem.iter,:) = [e_y,e_psi];
            assignin('base','stats',stats);
            assignin('base','controls_MPC',controls_MPC);
            assignin('base','state_sim',state_sim);
            assignin('base','y_sim',y_sim);
            assignin('base','constraints',constraints);
            assignin('base','computed_errors',computed_errors);
            assignin('base','state_cosim',state_cosim);
        end
%     end


end

