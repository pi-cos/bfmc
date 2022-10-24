
restoredefaultpath; clear all; close all; clc;

%% init 

X0 = 1;
Y0 = 1.5;
PSI0 = deg2rad(-35);

%%
s_step = 0.1;

PSI_orig = [PSI0:0.05:deg2rad(75),deg2rad(75):-0.05:deg2rad(-120)];
pos_orig = [X0;Y0];

for ii=1:length(PSI_orig)-1
    Xplus = pos_orig(1,ii)+s_step*cos(PSI_orig(ii));
    Yplus = pos_orig(2,ii)+s_step*sin(PSI_orig(ii));
    posplus = [Xplus;Yplus];
    pos_orig = [pos_orig,posplus];
end

%%

rotMat = @(psi) [cos(psi) -sin(psi); sin(psi), cos(psi)];
%%

pos_rot = [X0-X0;Y0-Y0];
PSI_rot = PSI_orig-PSI0;

for ii=2:length(PSI_orig)
    Xt = pos_orig(1,ii)-X0;
    Yt = pos_orig(2,ii)-Y0;
    posrotplus = rotMat(-PSI0)*[Xt;Yt];
%     posrotplus = [Xrot_plus;Yrot_plus];
    pos_rot = [pos_rot,posrotplus];
end

%%

figure; grid on; hold on; plot(pos_orig(1,:),pos_orig(2,:))
plot(pos_rot(1,:),pos_rot(2,:)); axis equal;
legend('orig','rot')

