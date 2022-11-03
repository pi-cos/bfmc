function [x, y, z, width, lat_inc, speed, s, crv] = readdrd_closed_smooth(drd_filename)

% readdrd(drd_filename)
%
%    read a drd2 road file.
% 
% Syntax:
%
%     [x, y, z, width, lat_inc, speed, s, crv] = readdrd(drd_filename)
%
% 2004-06-22 Davide Bacchet (davide.bacchet@vi-grade.com)
% Last Modified 2004-06-22

[x, y, width, lat_inc, speed, z] = textread(drd_filename, '%f %f %f %f %f %f', ...
                                            'headerlines', 1);

x(end) = x(1);
y(end) = y(1);
x = smooth(x,20);
y = smooth(y,20);

% arclength
dS = sqrt( diff(x).^2 + diff(y).^2 );
s = [0; cumsum(dS)];

% calculate curvature
Dx = diff(x)./dS;
Dx = [Dx; Dx(end)];
DDx = x(3:end) - 2*x(2:end-1) + x(1:end-2);
DDx = [DDx(1); DDx; DDx(end)]./([dS; dS(end)].^2);
Dy = diff(y)./dS;
Dy = [Dy; Dy(end)];
DDy = y(3:end) - 2*y(2:end-1) + y(1:end-2);
DDy = [DDy(1); DDy; DDy(end)]./([dS; dS(end)].^2);
crv = ( Dx.*DDy - Dy.*DDx ) ./ ( Dx.^2 + Dy.^2 ).^(1.5);

end
