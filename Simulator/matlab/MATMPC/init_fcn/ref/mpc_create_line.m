function [track] = mpc_create_line(tot_length,v,interp_step, extraNum)
%MPC_CREATE_CRICLE Summary of this function goes here
%   Detailed explanation goes here

ss = (0:interp_step:tot_length)';

s_length = length(ss);

psii = zeros(s_length,1);
xx = ss;
yy = zeros(s_length,1);

kk = zeros(s_length,1);
vv = v*ones(s_length,1);
ww = 0.5*ones(s_length,1);

ss = [ss; (ss(end):interp_step:ss(end)+extraNum*interp_step)'];
xx = [xx; (xx(end):interp_step:xx(end)+extraNum*interp_step)'];
yy = [yy; yy(end)*ones(extraNum+1,1)];
vv = [vv; vv(end)*ones(extraNum+1,1)];
kk = [kk; 0*ones(extraNum,1)];
psii = [psii; psii(end)*ones(extraNum+1,1)];
ww = [ww; ww(end)*ones(extraNum+1,1)];

track.s = ss;
track.X = xx;
track.Y = yy;
track.v = vv;
track.k = kk;
track.psi = psii;
track.w = ww;

end

