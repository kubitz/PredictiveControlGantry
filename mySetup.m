function [ param ] = mySetup(shape)
%% Modify the following function for your setup function

% This is a sample static K matrix for the controller
param.K = [2, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 2, 0, 0, 0, 0, 0];

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);

param.target=shape.target;
param.start=shape.start;
param.constraints=shape.constraints;
param.eps_r=shape.eps_r;
param.eps_t=shape.eps_t;
param.Wmax=shape.Wmax;
param.Tf=shape.Tf;
param.angleConstraint=3*pi/180;
param.Ts = 0.05;
param.Tf=shape.Tf;
N=20;

% Declare penalty matrices:
Q = 1*diag(1:8);
P = 100000*diag(3:10);
R = diag([2;3]);

load('Crane_NominalParameters.mat');
[param.A,param.B,param.C,~] = genCraneODE(m,M,MR,r,9.81,Tx,Ty,Vx,param.Ts);
%% Declare contraints
clAngle=[-param.angleConstraint;  -param.angleConstraint];
chAngle=[param.angleConstraint;  param.angleConstraint];
DAngle=zeros(2,8);DAngle(1,5)=1;DAngle(2,7)=1;
DAngle
[DRect,clRect,chRect]=getRectConstraints(param.constraints.rect)
% constrained vector is Dx, hence
D=[DAngle;DRect];
ch=[chAngle;chRect];
cl=[clAngle;clRect];
ul=[-1; -1];
uh=[1; 1];

%% Compute stage constraint matrices and vector
[Dt,Et,bt]=genStageConstraints(param.A,param.B,D,cl,ch,ul,uh);

%% Compute trajectory constraints matrices and vector
[param.DD,param.EE,param.bb]=genTrajectoryConstraints(Dt,Et,bt,N);

%% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(param.A,param.B,N);
[param.F,param.J,param.L]=genConstraintMatrices(param.DD,param.EE,Gamma,Phi,N);

%% Compute QP cost matrices
[param.H,param.G] = genCostMatrices(Gamma,Phi,Q,R,P,N);       
% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
[Lchol,p] = chol(param.H,'lower');
param.Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

end % End of mySetup



