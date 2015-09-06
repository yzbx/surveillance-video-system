%global config value
clc,clear;
root='D:\Program\matlab\surveillance-video-system';

dataset=true;
if(dataset==false)
    dataset='D:\Program\matlab\bgslibrary_mfc\dataset\objects';
    createDataset(dataset);
end

dataset='objectSetRecognize.mat';

target=['D:\Program\matlab\bgslibrary_mfc\dataset\objects','\video0-obj3-frame273.png'];
objectSetReconize(target,dataset);

% syms x r t l u a q e;
% %l lambda
% %q xigma
% %e kasin
% du=int(2*pi*l*u*exp(-pi*l*u^2)*q/(q+q*t*a^(1-e)*u^(a*e)*x^(-a)),u,0,inf)
% dx=int((1-du*x),x,r,inf)
% dr=int(exp(-2*pi*l*dx)*2*pi*l*r*exp(-pi*l*r^2),r,0,inf)
% dt=int(dr,t,0,inf)