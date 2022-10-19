
%% loop

while time(end) < Tf % current_state.s

    % find current pos wrt track
    X = state_sim(end,1);
    Y = state_sim(end,2);
    PSI = state_sim(end,3);
    search_indexes = (1:length(track.s)-(settings.N+1));
    squared_dist = (track.X(search_indexes)-X).^2 + (track.Y(search_indexes)-Y).^2;
    [~, curr_ref_index] = min(squared_dist);
%     curr_ref_index = curr_ref_index+1*ref_div;
    if last_ref_index - curr_ref_index > 0
        disp('Lap finished, break.')
        break
    end
    last_ref_index = curr_ref_index;

    % compute current errors wrt track centerline
    current_state.s = track.s(curr_ref_index);%time(end);
    current_state.X = X;
    current_state.Y = Y;
    current_state.psi = PSI;
    [e_y, e_psi] = errors_calculator(current_state, curr_ref_index, track);
    computed_errors(mem.iter,:) = [e_y,e_psi];

%     % update state
    input.x0 = [X;Y;PSI;e_y;e_psi];
%     state_sim(end,:) = input.x0;
    % obtain the state measurement
%     input.x0 = state_sim(end,:)';
        
    % the reference input.y is a ny by N matrix
    % the reference input.yN is a nyN by 1 vector    
    input.y = repmat(REF',1,settings.N);
    input.yN = REF(1:settings.nyN)';
    % curvature
    samples = curr_ref_index:ref_div:curr_ref_index+settings.N*ref_div;
    samples_mod = mod(samples,length(track.s)-(settings.N+1));
    if any(samples ~= samples_mod)
        idx_zero = find(samples_mod==0);
        samples_mod(idx_zero:end)=samples_mod(idx_zero:end)+1;
        samples = samples_mod;
    end
    input.od(1,:) = track.k(samples);
    % velocity
    input.y(3,:) = track.v(samples(1:end-1));

%     input.od(1,:) = zeros(settings.N+1,1);
%     input.y(3,:) = 0.1*ones(settings.N,1);
%     input.y(4,:) = deg2rad(5)*sin(time(end)*3)*ones(settings.N,1);
    
    % call the NMPC solver 
    [output, mem] = mpc_nmpcsolver(input, settings, mem, opt);
        
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
            
            % for CMoN-RTI
%             mem.A=[mem.A(:,nx+1:end),mem.A(:,end-nx+1:end)];
%             mem.B=[mem.B(:,settings.nu+1:end),mem.B(:,end-settings.nu+1:end)];
%             mem.F_old = [mem.F_old(:,2:end),mem.F_old(:,end)];
%             mem.V_pri = [mem.V_pri(:,2:end),mem.V_pri(:,end)];
%             mem.V_dual = [mem.V_dual(:,2:end),mem.V_dual(:,end)];
%             mem.q_dual = [mem.q_dual(:,2:end),mem.q_dual(:,end)];            
%             mem.shift_x = input.x-output.x;
%             mem.shift_u = input.u-output.u;
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
    
    % Simulate system dynamics
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(:,1);
    sim_input.z = input.z(:,1);
    sim_input.p = input.od(:,1);

    [xf, zf] = Simulate_System(sim_input.x, sim_input.u, sim_input.z, sim_input.p, mem, settings);
    xf = full(xf);
    
    % Collect outputs
    y_sim = [y_sim; full(h_fun('h_fun', xf, sim_input.u, sim_input.p))'];  
    
    % Collect constraints
    constraints=[constraints; full( path_con_fun('path_con_fun', xf, sim_input.u, sim_input.p) )'];
        
    % store the optimal solution and states
    controls_MPC = [controls_MPC; output.u(:,1)'];
    state_sim = [state_sim; xf'];
    KKT= [KKT;OptCrit];
    OBJ= [OBJ;output.info.objValue];
    CPT = [CPT; cpt, tshooting, tcond, tqp];
    numIT = [numIT; output.info.iteration_num];
    
    % go to the next sampling instant
    nextTime = mem.iter*settings.Ts_st; 
    mem.iter = mem.iter+1;
    if mod(mem.iter,10)==1
    disp(['current space:' num2str(nextTime) 'm  CPT:' num2str(cpt) 'ms  SHOOTING:' num2str(tshooting) 'ms  COND:' num2str(tcond) 'ms  QP:' num2str(tqp) 'ms  Opt:' num2str(OptCrit) '   OBJ:' num2str(OBJ(end)) '  SQP_IT:' num2str(output.info.iteration_num)]);  
    end
    time = [time nextTime];   
end

%% post run

computed_errors(mem.iter,:) = [e_y,e_psi];

if strcmp(opt.qpsolver, 'qpoases')
    qpOASES_sequence( 'c', mem.warm_start);
end
% if strcmp(opt.qpsolver, 'qpalm')
%     mem.qpalm_solver.delete();
% end
clear mex;

%%

disp(['Average CPT: ', num2str(mean(CPT(2:end,:),1)) ]);
disp(['Maximum CPT: ', num2str(max(CPT(2:end,:))) ]);