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
% Create the output array of the appropriate size
u = zeros(2,1);
u = genMPController(param.Linv,param.G,param.F,param.bb,param.J,param.L,x_hat,r,2);
end % End of myMPController

function [F,J,L] = genConstraintMatrices(DD,EE,Gamma,Phi,N)
sx = size(DD,2) / N;
shift = [zeros(sx, sx*(N-1));eye(sx*(N-1))]*[eye(sx*(N-1)), zeros(sx*(N-1),sx)];
F = DD*shift*Gamma + EE;
J = -DD*(shift*Phi + [eye(sx);zeros(sx*(N-1), sx)]);
L =  - DD * kron(ones(N,1), eye(sx)) - J;
end

function [Gamma,Phi] = genPrediction(A,B,N)
% GENPREDICTION  [Gamma,Phi] = genPrediction(A,B,N). 
% A and B are discrete-time state space matrices for x[k+1]=Ax[k]+Bu[k]
% N is the horizon length. 
% Your code is suppose to work for any linear system, not just the gantry crane. 

% Write your code here
sizeA=size(A,1);
A_big=kron(eye(N),-A);
B_big=kron(eye(N),B);

A_big=[zeros(sizeA,N*sizeA);A_big];
A_big=[A_big,zeros((N+1)*sizeA,sizeA)];
A_big=A_big+eye((N+1)*sizeA);
B_big=[zeros(size(B,1),size(B,2)*N);B_big];
Gamma=A_big\B_big;
Gamma=Gamma(sizeA+1:end,:);
I=[eye(sizeA);zeros(N*sizeA,sizeA)];
Phi=A_big\I;
Phi=Phi(size(B,1)+1:end,:);

end

function [A,B,C,D] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts)
% Inputs:
% m = Pendulum mass (kg)
% M = Cart mass (kg)
% MR = Rail mass (kg)
% r = String length (m)
% g = gravitational accelaration (m*s^-2)
% Tx = Damping coefficient in X direction (N*s*m^-1)
% Ty = Damping coefficient in Y direction (N*s*m^-1)
% Vm = Input multiplier (scalar)
% Ts = Sample time of the discrete-time system (s)
% Outputs:
% A,B,C,D = State Space matrices of a discrete-time or continuous-time state space model

% The motors in use on the gantry crane are identical and therefore Vx=Vy.
Vx=Vm;
Vy=Vm;

 
A =[
 
[0,                 1, 0,         0,                                0, 0,                  0, 0]
[0,    -Tx/(M + MR), 0,         0,                  (g*m)/(M + MR), 0,                  0, 0]
[0,                 0, 0,         1,                                0, 0,                  0, 0]
[0,                 0, 0,    -Ty/M,                                0, 0,            (g*m)/M, 0]
[0,                 0, 0,         0,                                0, 1,                  0, 0]
[0, Tx/(r*(M + MR)), 0,         0, -(g*(M + MR + m))/(r*(M + MR)), 0,                  0, 0]
[0,                 0, 0,         0,                                0, 0,                  0, 1]
[0,                 0, 0, Ty/(M*r),                                0, 0, -(g*(M + m))/(M*r), 0]
 ];

B =  [
 
[                 0,          0]
[     Vx/(M + MR),          0]
[                 0,          0]
[                 0,      Vy/M]
[                 0,          0]
[-Vx/(r*(M + MR)),          0]
[                 0,          0]
[                 0, -Vy/(M*r)]
 
];

C=eye(8);
D=zeros(8,2);

% if Ts>0 then sample the model with a zero-order hold (piecewise constant) input, otherwise return a continuous-time model
if Ts>0
%     Return discrete-time SS model matrices
    tmp_sys=ss(A,B,C,D);
    sys_d=c2d(tmp_sys,Ts);
    [A,B,C,D] = ssdata(sys_d);
    return
end
   return
end


function [H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N)
%% cost function matrices
% Gamma and Phi are the prediction matrices
% Q is the stage cost weight on the states, i.e. x'Qx
% R is the stage cost weight on the inputs, i.e. u'Ru
% P is the terminal weight on the final state

    % Your code goes here
    V=kron(eye(N-1),Q);
    bigR=kron(eye(N),R);
    V=blkdiag(V,P);
    G=2*Gamma.'*V*Phi;
    H=2*(Gamma.'*V*Gamma+bigR);
end


function [u,status,iA1] = genMPController(H,G,F,bb,J,L,x,xTarget,m)
    % H       - quadratic term in the cost function (Linv if using mpcActiveSetSolver).
    % G       - matrix in the linear term in the cost function.
    % F       - LHS of the inequalities term.
    % bb      - RHS of the inequalities term.
    % J       - RHS of inequalities term.
    % L       - RHS of inequalities term. Use this to shift constraints around target point
    % x       - current state
    % xTarget - target equilibrium point.
    % m       - Number of inputs.
    % iA      - active inequalities, see doc mpcqpsolver
    %
    % u is the first input vector u_0 in the sequence [u_0; u_1; ...; u_{N-1}]; 
    % In other words, u is the value of the receding horizon control law
    % evaluated with the current state x0 and target xTarget

    % Please read the documentation on mpcActiveSetSolver to understand how it is
    % suppose to be used. Use iA and iA1 to pass the active inequality vector 

    opt.MaxIterations = 200;
    opt.IntegrityChecks = false;%% for code generation
    opt.ConstraintTolerance = 1e-3;
    opt.DataType = 'double';
    opt.UseHessianAsInput = false;


    % Compute the linear term of the cost function
    linMul = G*(x - xTarget);
    % Compute the matrix A and b for the inequality constraint formulation
    Acon = F;
    sJ=size(J);
    sx=size(x);
    sL=size(L);
    sT=size(xTarget);
    RHScon = J*x + L*xTarget + bb;
    
    [U,status,iA1]=mpcActiveSetSolver(H,linMul,Acon,RHScon,[],zeros(0,1),false(size(bb)),opt);
    u=U(1:m);
end

function [Dt,Et,bt] = genStageConstraints(A,B,D,cl,ch,ul,uh)
    sU=size(ul,1);
    sA=size(A,2);
    sB=size(B,2);
    pD=D*A;
    Dt=[pD;-pD;zeros(sU,sA);zeros(sU,sA)];
    pE=D*B;
    Et=[pE;-pE;eye(sU);-eye(sU)];
    bt=[ch;-cl;uh;-ul];
end

function [DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N)
DD=kron(eye(N),Dt);
EE=kron(eye(N),Et);
bb=kron(ones(N,1),bt);
end

function [DRect,clRect,chRect] = getRectConstraints(rect)
A = rect(1,:);
B = rect(2,:);
C = rect(3,:);
D = rect(4,:);
% First two parallel edges - computed in same direction
[a1,b1,c1] = getLineParamsStd(A,B);
[a2,b2,c2] = getLineParamsStd(D,C);
% Second set of parallel edges - computed in same direction
[a3,b3,c3] = getLineParamsStd(B,C);
[a4,b4,c4] = getLineParamsStd(A,D);
% Compute D matrix and upper/lower bounds
DRect=zeros(2,8);DRect(1,1)=a1;DRect(1,3)=b1;DRect(2,1)=a3;DRect(2,3)=b3;
clRect=[c2;c4];
chRect=[c1;c3];
end

function [A,B,C] = getLineParamsStd(pointA,pointB)
% Get line parameters in standard form Ax+By=C
A=pointA(2)-pointB(2);
B=pointB(1)-pointA(1);
C=A*pointA(1)+B*pointA(2);
end
