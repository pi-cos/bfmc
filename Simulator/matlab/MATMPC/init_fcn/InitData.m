%% Initialize Data
function [input] = InitData(settings)

    nx = settings.nx;       % No. of differential states
    nu = settings.nu;       % No. of controls
    nz = settings.nz;       % No. of algebraic states
    ny = settings.ny;        % No. of outputs (references)    
    nyN= settings.nyN;       % No. of outputs at terminal stage 
    np = settings.np;        % No. of parameters (on-line data)
    nc = settings.nc;        % No. of constraints
    ncN = settings.ncN;      % No. of constraints at terminal stage
    N  = settings.N;         % No. of shooting points
    nbx = settings.nbx;      % No. of state bounds
    nbu = settings.nbu;      % No. of control bounds
    nbu_idx = settings.nbu_idx;  % Index of control bounds

    switch settings.model
                      
        case 'InvertedPendulum'
            input.x0 = [0;pi;0;0];    
            input.u0 = zeros(nu,1); 
            input.z0 = zeros(nz,1);
            para0 = 0;  

            Q=repmat([10 10 0.1 0.1 0.01]',1,N);
            QN=[10 10 0.1 0.1]';

            % upper and lower bounds for states (=nbx)
            lb_x = -2;
            ub_x = 2;

            % upper and lower bounds for controls (=nbu)           
            lb_u = -20;
            ub_u = 20;
                       
            % upper and lower bounds for general constraints (=nc)
            lb_g = [];
            ub_g = [];            
            lb_gN = [];
            ub_gN = [];

        case 'KinematicVehicle'
            input.x0 = zeros(nx,1);
            input.u0 = zeros(nu,1);
            input.u0(1) = 0.1;
            input.z0 = zeros(nz,1);
            para0 = [0;0.2;0.25];

            Q=repmat([1 1 0.01 0.01]',1,N);
            QN=[1 1]';

            % upper and lower bounds for states (=nbx)
            lb_x = [-0.5;-pi/2];
            ub_x = [+0.5;+pi/2];

            % upper and lower bounds for controls (=nbu)
            lb_u = [ 0;-deg2rad(45)]; %
            ub_u = [+2;+deg2rad(45)]; %

            % upper and lower bounds for general constraints (=nc)
            lb_g = [];
            ub_g = [];
            lb_gN = [];
            ub_gN = [];
                                                            
    end

    % prepare the data
    
    input.lb = repmat(lb_g,N,1);
    input.ub = repmat(ub_g,N,1);
    input.lb = [input.lb;lb_gN];
    input.ub = [input.ub;ub_gN];
            
    lbu = -inf(nu,1);
    ubu = inf(nu,1);
    for i=1:nbu
        lbu(nbu_idx(i)) = lb_u(i);
        ubu(nbu_idx(i)) = ub_u(i);
    end
                
    input.lbu = repmat(lbu,1,N);
    input.ubu = repmat(ubu,1,N);
    
    input.lbx = repmat(lb_x,1,N);
    input.ubx = repmat(ub_x,1,N);
        
    x = repmat(input.x0,1,N+1);  % initialize all shooting points with the same initial state
    u = repmat(input.u0,1,N);    % initialize all controls with the same initial control
    z = repmat(input.z0,1,N);    % initialize all algebraic state with the same initial condition
    para = repmat(para0,1,N+1);  % initialize all parameters with the same initial para
         
    input.x=x;           % (nx by N+1)
    input.u=u;           % (nu by N)
    input.z=z;           % (nz by N)
    input.od=para;       % (np by N+1)
    input.W=Q;           % (ny by N)
    input.WN=QN;         % (nyN by 1)
     
    input.lambda=zeros(nx,N+1);   % langrangian multiplier w.r.t. equality constraints
    input.mu=zeros(N*nc+ncN,1);   % langrangian multipliers w.r.t. general inequality constraints
    input.mu_u = zeros(N*nu,1);   % langrangian multipliers w.r.t. input bounds
    input.mu_x = zeros(N*nbx,1);  % langrangian multipliers w.r.t. state bounds
    
end