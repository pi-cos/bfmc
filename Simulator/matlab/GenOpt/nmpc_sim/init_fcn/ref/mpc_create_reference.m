function Ref = mpc_create_reference(drd_filename_in, interp_step, extraNum)

% create a reference trajectory, comprehensive of time
% the trajectory has the format
%       R.s
%       R.X
%       R.Y
%       R.v
%       R.k
%       R.psi

if nargin<2
    interp_step = 1;
    extraNum = 100;
end

% create a reference trajectory from a drd file
[x, y, ~, w, ~, v, s, crv] = readdrd(drd_filename_in);

x = -x;
y = -y;

psi = zeros(length(s),1);
for ii = 2:length(s)-1
   psi(ii) = atan2((y(ii+1)-y(ii-1)),(x(ii+1)-x(ii-1)));
end
psi(end) = psi(end-1);
psi = unwrap(psi);
% psi_2 = unwrap([0;atan2((y(2:end)-y(1:end-1)),(x(2:end)-x(1:end-1)))]);

ss = [s(1):interp_step:s(end)]';
xx = interp1(s,x,ss,'spline','extrap');
yy = interp1(s,y,ss,'spline','extrap');
vv = interp1(s,v,ss,'spline','extrap');
crvv = interp1(s,crv,ss,'spline','extrap');
psii = interp1(s,psi,ss,'spline','extrap');
ww = interp1(s,w,ss,'spline','extrap');

% psii = zeros(length(ss),1);
% for ii = 2:length(ss)-1
%    psii(ii) = atan2((yy(ii+1)-yy(ii-1)),(xx(ii+1)-xx(ii-1)));
% end
% psii(end) = psii(end-1);
% psii = unwrap(psii);

% add extraNum [m] for the ending
ss = [ss; (ss(end):interp_step:ss(end)+extraNum*interp_step)'];
xx = [xx; (xx(end):interp_step:xx(end)+extraNum*interp_step)'];
yy = [yy; yy(end)*ones(extraNum+1,1)];
vv = [vv; vv(end)*ones(extraNum+1,1)];
crvv = [crvv; crvv(end)*ones(extraNum+1,1)];
psii = [psii; psii(end)*ones(extraNum+1,1)];
ww = [ww; ww(end)*ones(extraNum+1,1)];

Ref.s = ss;
Ref.X = xx;
Ref.Y = yy;
Ref.v = vv;
Ref.k = crvv;
Ref.psi = psii;
Ref.w = ww;

end

