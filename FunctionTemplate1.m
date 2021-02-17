function [ param ] = mySetup(shape)
%% Modify the following function for your setup function

% Copy course parameters
param.constraints = shape.constraints;
param.tolerances = shape.tolerances; 
param.eps_r = shape.eps_r;      
param.eps_t = shape.eps_t;      
param.start = shape.start;      
param.target = shape.target;     
param.Wmax = shape.Wmax;       
param.Tf = shape.Tf;    

% This is how to set the sampling interval
param.Ts = 0.05;

% Load model parameters (NOTE! These are not precise as they are later
% perturbed)
load('Crane_NominalParameters.mat');
[param.A,param.B,param.C,~] = genCraneODE(m,M,MR,r,9.81,Tx,Ty,Vx,Vy,param.Ts);


% This is a sample static K matrix for the controller
%param.K = [2, 0, 0, 0, 0, 0, 0, 0;
%           0, 0, 2, 0, 0, 0, 0, 0];

% This is a sample way to send reference points
param.xTar = shape.target(1);
param.yTar = shape.target(2);




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
%%

%horizon length
N = 15;

%u = param.K*(r - x_hat);

% Declare penalty matrices:
Q = 1*diag(1:8);
P = 100000*diag(3:10);
R = diag([2;3]);

A = param.A;
B = param.B;
C = param.C;

% Constraints %SHOULD THE TARGET BE SPECIFIED AS A CONSTRAINT?
[DRect,chRect,clRect] = rectConstraints(param.constraints.rect);
D = zeros(2, size(param.A,1));
D(1,1) = DRect(1,1);
D(2,1) = DRect(2,1);
D(1,3) = DRect(1,2);
D(2,3) = DRect(2,2);

% Compute stage constraint matrices and vector
cl = clRect;
ch = chRect;
ul = [-1; -1];
uh = [1; 1];
[Dt,Et,bt] = genStageConstraints(A,B,D,cl,ch,ul,uh);

% Compute trajectory constraints matrices and vector
[DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N);

% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N);
[F,J,L]=genConstraintMatrices(DD,EE,Gamma,Phi,N);
% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);       
% Prepare cost and constraint matrices for mpcActiveSetSolver
% See doc for mpcActiveSetSolver
[Lchol,p] = chol(H,'lower');
Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true));

% Run a linear simulation to test your genMPController function
u = genMPController(Linv,G,F,bb,J,L,x_hat,r,2);

end % End of myMPController







%% OTHER FUNCTIONS

function [A,B,C,D] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vx,Vy,Ts)
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
%Vx=Vm;
%Vy=Vm;

% replace A,B,C,D with the correct values
A=[ 0, 1,             0, 0,        0,                          0, 0,              0;
    0, -Tx/(M+MR),    0, 0,        g*m/(M+MR),                 0, 0,              0;
    0, 0,             0, 1,        0,                          0, 0,              0;
    0, 0,             0, -Ty/M,    0,                          0, g*m/M,          0;
    0, 0,             0, 0,        0,                          1, 0,              0;
    0, Tx/(r*(M+MR)), 0, 0,        -(g*(m+M+MR))/(r*(M+MR)),   0, 0,              0;
    0, 0,             0, 0,        0,                          0, 0,              1;
    0, 0,             0, Ty/(M*r), 0,                          0, -g*(m+M)/(M*r), 0 ];
B=[ 0,              0;
    Vx/(M+MR),      0;
    0,              0;
    0,              Vy/M;
    0,              0;
    -Vx/(r*(M+MR)), 0; 
    0,              0; 
    0,              -Vy/(M*r) ];
C=eye(8);
D=zeros(8,2);

% if Ts>0 then sample the model with a zero-order hold (piecewise constant) input, otherwise return a continuous-time model
if Ts>0
    sys = ss(A,B,C,D);
    sysd = c2d(sys, Ts);
    A = sysd.A;
    B = sysd.B;
    C = sysd.C;
    D = sysd.D;
end

end

function [a,c] = lineParams(pointA, pointB)
    % given two points, returns the parameters of the line ax+y=c
    dy = pointB(2)-pointA(2);
    dx = pointB(1)-pointA(1);
    a = - dy/dx; % y + a * x = c
    c = pointA(2) + a * pointA(1);
end

function [mat, ch, cl] = rectConstraints(rect)
    A = rect(1,:);
    B = rect(2,:);
    C = rect(3,:);
    D = rect(4,:);
    [a1,c1] = lineParams(A,D);
    [a2,c2] = lineParams(A,B);
    [a3,c3] = lineParams(B,C);
    [a4,c4] = lineParams(D,C);
    
    mat  = [ a1 , 1;
             a2 , 1 ];% CHECK THIS MIGHT BE WRONG!!!
         
    ch = [ c3; c2 ];
    cl = [ c1; c4 ];
end




function u = genMPController(H,G,F,bb,J,L,x,xTarget,m)
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
%% your code starts here
linTerm = G * (x - xTarget);
rightIneqConstr = bb + J*x + L*xTarget;
sL=size(L)
sT=size(xTarget)
[U,~] = mpcActiveSetSolver(H, linTerm, F, rightIneqConstr, [], zeros(0,1), false(size(bb)), opt); 
%CEHCK IA CONTRAINT THING
%% your remaining code here
u = U(1:m);
end



function [Dt,Et,bt] = genStageConstraints(A,B,D,cl,ch,ul,uh)
    Dt = [D*A; -D*A; zeros(2*length(ul),size(D*A,2))];
    Et = [D*B; -D*B; eye(size(D*B,2)); -eye(size(D*B,2))];
    bt = [ch; -cl; uh; -ul];
end

function [DD,EE,bb] = genTrajectoryConstraints(Dt,Et,bt,N)

% your code goes here
DD = kron(eye(N), Dt);
EE = kron(eye(N), Et);
bb = kron(ones(N,1), bt);

end


function [Gamma,Phi] = genPrediction(A,B,N)
% GENPREDICTION  [Gamma,Phi] = genPrediction(A,B,N). 
% A and B are discrete-time state space matrices for x[k+1]=Ax[k]+Bu[k]
% N is the horizon length. 
% Your code is suppose to work for any linear system, not just the gantry crane. 

% Write your code here
n_states = size(A,1);
n_inputs = size(B,2);

bigA = eye(n_states*(N+1));
bigA(n_states+1:end,1:end-n_states) = bigA(n_states+1:end,1:end-n_states) + kron(eye(N),-A);

bigB = zeros(n_states*(N+1),n_inputs*N);
bigB(n_states+1:end,1:end) = kron(eye(N),B);

extender = [eye(n_states); zeros(n_states*N,n_states)];

Phi = bigA\extender;
Phi = Phi(n_states+1:end,1:end);
Gamma = bigA\bigB;
Gamma = Gamma(n_states+1:end,1:end);

end


function [F,J,L] = genConstraintMatrices(DD,EE,Gamma,Phi,N)

% your code goes here
n_states = size(DD,2) / N;

F = EE + DD * [eye(N*n_states), zeros(N*n_states, n_states)] * [zeros(n_states,size(Gamma,2)); Gamma];
J = - DD * [eye(N * n_states), zeros(N*n_states, n_states)] * [eye(size(Phi,2)); Phi];
L =  - DD * kron(ones(N,1), eye(n_states)) - J;

end



function [H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N)
%% cost function matrices
% Gamma and Phi are the prediction matrices
% Q is the stage cost weight on the states, i.e. x'Qx
% R is the stage cost weight on the inputs, i.e. u'Ru
% P is the terminal weight on the final state

% Your code goes here
extendedGamma = [zeros(size(Phi,2), size(Gamma,2)); Gamma];
bigQ = kron(eye(N+1), Q);
bigQ(end-size(P,1)+1:end, end-size(P,2)+1:end) = P;
bigR = kron(eye(N),R);
H = (extendedGamma'*bigQ*extendedGamma + bigR) * 2;

extendedPhi = [eye(size(Phi,2)); Phi];
G = (extendedGamma'*bigQ'*extendedPhi) * 2;

end