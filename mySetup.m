function [ param ] = mySetup(shape)
%% Modify the following function for your setup function
clear myMPController;
% This is a sample static K matrix for the controller
param.K = [2, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 2, 0, 0, 0, 0, 0];

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);
param.target=shape.target;r = zeros(8,1);

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
param.Ts = 0.05;
param.useShrinkingHorizon=true;
param.N=ceil(param.Tf/param.Ts);

% Declare penalty matrices:
Qt = [0 10 0 10 0 0 0 0];
Pt = [0 0 0 0 0 0 0 0];
Rt = [1 1];

param.Q = 1*diag(Qt);
param.P = 100*diag(Pt);
param.R = 1*diag(Rt);
param.numStates=size(param.Q,1);
param.numInputs=size(param.R,1);


load('Crane_NominalParameters.mat');
[param.A,param.B,param.C,~] = genCraneODEmod(m,M,MR,r,9.81,Tx,Ty,Vx,Vy,param.Ts);
%% Declare contraints
clAngle=[-param.angleConstraint;  -param.angleConstraint];
chAngle=[param.angleConstraint;  param.angleConstraint];
DAngle=zeros(2,8);DAngle(1,5)=1;DAngle(2,7)=1;

[DRectL,clRectL,chRectL]=getRectConstraintsLoad(param.constraints.rect,r);
[DRect,clRect,chRect]=getRectConstraints(param.constraints.rect);
% constrained vector is Dx, hence
D=[DAngle;DRectL;DRect];
ch=[chAngle;chRectL;chRect];
cl=[clAngle;clRectL;clRect];
ul=[-1; -1];
uh=[1; 1];

%% Compute stage constraint matrices and vector
[Dt,Et,bt]=genStageConstraints(param.A,param.B,D,cl,ch,ul,uh);
param.numConstraints=size(Dt,1)
%% Compute trajectory constraints matrices and vector
[param.DD,param.EE,param.bb]=genTrajectoryConstraints(Dt,Et,bt,param.N);
sDD=size(param.DD,2);

% param.EE=[param.EE(1:end-32,:);zeros(32,size(param.EE,2))];
% uc=param.targetMod+0.5*param.eps_r;
% lc=-param.targetMod+0.5*param.eps_r;
% param.bb=[param.bb(1:end-32,:);uc;lc;uc;lc];
% 
% param.DD=[param.DD(1:end-32,:);zeros(8,sDD-16),eye(8),zeros(8,8);zeros(8,sDD-16),-eye(8),zeros(8,8)];
% param.DD=[param.DD;zeros(8,sDD-8),eye(8);zeros(8,sDD-8),-eye(8)];

param.DD=[param.DD(1:end-16,:);zeros(8,sDD-8),eye(8);zeros(8,sDD-8),-eye(8)];
param.EE=[param.EE(1:end-16,:);zeros(16,size(param.EE,2))];
param.bb=[param.bb(1:end-16,:);param.targetMod+0.5*param.eps_r;-param.targetMod+0.5*param.eps_r];
%% Compute QP constraint matrices
[param.Gamma,param.Phi] = genPrediction(param.A,param.B,param.N);
[param.F,param.J,param.L]=genConstraintMatrices(param.DD,param.EE,param.Gamma,param.Phi,param.N);

%% Compute QP cost matrices
[param.H,param.G] = genCostMatrices(param.Gamma,param.Phi,param.Q,param.R,param.P,param.N);       
% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
[Lchol,p] = chol(param.H,'lower');
param.Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

end % End of mySetup



