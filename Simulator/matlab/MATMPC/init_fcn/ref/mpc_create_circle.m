function [track] = mpc_create_circle(R,v,interp_step, extraNum)
%MPC_CREATE_CRICLE Summary of this function goes here
%   Detailed explanation goes here

ss = (0:interp_step:2*pi*R)';

psii = (0:interp_step/R:2*pi)';
xx = R*cos(psii-pi/2);
yy = R+R*sin(psii-pi/2);

kk = 1/R*ones(length(ss),1);
vv = v*ones(length(ss),1);
ww = 0.5*ones(length(ss),1);

% psii = zeros(length(ss),1);
% for ii = 2:length(ss)-1
%    psii(ii) = atan2((yy(ii+1)-yy(ii-1)),(xx(ii+1)-xx(ii-1)));
% end
% psii(end) = psii(end-1);
% psii = unwrap(psii);

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

