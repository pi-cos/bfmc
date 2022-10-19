
%% loop
sim_iter = 0;
while time(end) < Tf % current_state.s

    % find current pos wrt track
    X = state_sim(end,1);
    Y = state_sim(end,2);
    PSI = state_sim(end,3);
    
    % call NMPC
    states = [X;Y;PSI];
    ctrls = nmpc_controller(states);
    if ctrls(end)
        break
    end
    
    % Simulate system dynamics
    sim_input.x = state_sim(end,:).';
    sim_input.u = ctrls(1:settings.nu);
    sim_input.z = input.z(:,1);
    sim_input.p = input.od(:,1);
    k = ctrls(3);
    sim_input.p(1) = k;

    [xf, zf] = Simulate_System(sim_input.x, sim_input.u, sim_input.z, sim_input.p, mem, settings);
    xf = full(xf);

    state_sim = [state_sim; xf'];
    
    % go to the next sampling instant
    sim_iter = sim_iter+1;
    nextTime = sim_iter*settings.Ts_st; 
    time = [time nextTime];   

end

%% post run

if strcmp(opt.qpsolver, 'qpoases')
    qpOASES_sequence( 'c', mem.warm_start);
end
% if strcmp(opt.qpsolver, 'qpalm')
%     mem.qpalm_solver.delete();
% end
clear mex;

%%

disp(['Average CPT: ', num2str(mean(stats.CPT(2:end,:),1)) ]);
disp(['Maximum CPT: ', num2str(max(stats.CPT(2:end,:))) ]);