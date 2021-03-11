function [ param ] = mySetup(shape)
%% Modify the following function for your setup function
clear myMPController;
% This is a sample static K matrix for the controller
param.K = [2, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 2, 0, 0, 0, 0, 0];

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);
param.target=shape.target;


% Make the crane go to (xTar, yTar)
r=zeros(8,1); r(1,1) = param.xTar; r(3,1) = param.yTar;
param.targetMod=r;
param.start=shape.start;
param.constraints=shape.constraints;
param.eps_r=shape.eps_r;
param.eps_t=shape.eps_t;
param.Wmax=shape.Wmax;
param.Tf=shape.Tf;
param.angleConstraint=2.5*pi/180;
param.Ts = 0.1;
param.N=20;



% Declare penalty matrices:
Qt = [0 10 0 10 0 0 0 0];
Pt = [0 0 0 0 0 0 0 0];
Rt = [1 1];

param.Q = 1*diag(Qt);
param.P = 100*diag(Pt);
param.R = 1*diag(Rt);
param.numStates=size(param.Q,1);
param.numInputs=size(param.R,1);

param.nx = 8;
param.ny = 8;
param.nu = 2;

load('Crane_NominalParameters.mat');
param.nlobj = nlmpc(param.nx,param.ny,param.nu);
param.nlobj.Model.StateFcn = @(x,u) crane_nl_model_mod(x, u, Tx, Ty ,Tl ,Vx ,Vy ,MR ,M ,m, r, Vl);
param.nlobj.Ts = param.Ts;
param.nlobj.PredictionHorizon = param.N;
param.nlobj.ControlHorizon = 20;
param.nlobj.Weights.OutputVariables = [10 0 10 0 1 0 1 0];

x0 = [-10;0;-10;0;0;0;0;0];  % robot parks at [-10, -10], facing north
u0 = zeros(param.nu,1);           % thrust is zero
validateFcns(param.nlobj,x0,u0);



 
end % End of mySetup



function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

% Create the output array of the appropriate size. This vector is passed
% into myMPController, so its size must match what is expected by the code
% there.
r = zeros(8,1);
% Make the crane go to (xTar, yTar)
r(1,1) = param.xTar;
r(3,1) = param.yTar;

end % End of myTargetGenerator

function x_hat = myStateEstimator(u, y, param)
%% Modify the following function for your state estimator (if desired)

% Create the output array of the appropriate size. This vector is passed
% into myMPController, so its size must match what is expected by the code
% there.
x_hat = zeros(8,1);

% By default, just pass the system measurements through
x_hat( 1:length(y),1 ) = y;

end % End of myStateEstimator


%% Modify the following function for your controller
function u = myMPController(r, x_hat, param)
%% Do not delete this line
u=zeros(param.numInputs,1);
% Create the output array of the appropriate size
persistent it lastMv
if isempty(it)
    it=0;
    lastMv = zeros(param.nu,1);
end 

if (it<param.N)
    N=param.N-it;
    u = nlmpcmove(param.nlobj,x_hat,lastMv,param.targetMod.')
    it=it+1;
    lastMv=u;
end 
end % End of myMPController


function [A,B,C] = getLineParamsStd(pointA,pointB)
% Get line parameters in standard form Ax+By=C
A=pointA(2)-pointB(2);
B=pointB(1)-pointA(1);
C=A*pointA(1)+B*pointA(2);
end


function dx = crane_nl_model_mod(x, u, Tx, Ty ,Tl ,Vx ,Vy ,MR ,M ,m, r,Vl)
dx = zeros(8, 1);
% Extract the crane parameters
% Tx = craneParams.Tx; % friction coefficient
% Ty = craneParams.Ty; % friction coefficient
% Tl = craneParams.Tl; % friction coefficient
% Vx = craneParams.Vx; % force coefficient
% Vy = craneParams.Vy; % force coefficient
% Vl = craneParams.Vl; % force coefficient
% MR = craneParams.MR; % mass of rail
% M  = craneParams.M; % mass of cart
% m  = craneParams.m; % mass of pendulum
% This we don't change - after all, all experiments are done on Earth :)
g  = 9.81;
% Extract the states
x1 = x(1); % x
x2 = x(2); % x dot
x3 = x(3); % y
x4 = x(4); % y dot
x5 = x(5); % theta
x6 = x(6); % theta dot
x7 = x(7); % phi
x8 = x(8); % phi dot
x9 = r; % l
x10 = 0; % l dot
% Compute the input forces
fx = Vx.*u(1);
fy = Vy.*u(2);
fl = Vl.*0;

% Compute the derivatives
dx(1, 1)  = x2;
dx(2, 1)  = (cos(x5) .* cos(x7) .^ 2 .* sin(x5) .* g .* m + sin(x5) .* (Tl .* x10 - fl) .* cos(x7) - Tx .* x2 + fx) ./ (M + MR);
dx(3, 1)  = x4;
dx(4, 1)  = ((m .* g .* cos(x5) .* cos(x7) + Tl .* x10 - fl) .* sin(x7) - Ty .* x4 + fy) ./ M;
dx(5, 1)  = x6;  
dx(6, 1)  = (-cos(x5) .^ 2 .* cos(x7) .^ 2 .* sin(x5) .* g .* m + (-sin(x5) .* (Tl .* x10 - fl) .* cos(x5) - 0.2e1 .* x6 .* x10 .* (M + MR)) .* cos(x7) + (Tx .* x2 - fx) .* cos(x5) + 0.2e1 .* (M + MR) .* (sin(x7) .* x6 .* x8 .* x9 - sin(x5) .* g ./ 0.2e1)) ./ x9 ./ cos(x7) ./ (M + MR);
dx(7, 1)  = x8;
dx(8, 1)  = (-cos(x5) .* g .* sin(x7) .* m .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 2 + ((-M .* (Tl .* x10 - fl) .* cos(x5) .^ 2 - M .^ 2 .* x6 .^ 2 .* x9 - x6 .^ 2 .* x9 .* MR .* M - MR .* (Tl .* x10 - fl)) .* sin(x7) - (-Ty .* x4 + fy) .* (M + MR)) .* cos(x7) - 0.2e1 .* ((g .* (M + MR) .* cos(x5) ./ 0.2e1 + sin(x5) .* (Tx .* x2 - fx) ./ 0.2e1) .* sin(x7) + x10 .* x8 .* (M + MR)) .* M) ./ x9 ./ M ./ (M + MR);

end
