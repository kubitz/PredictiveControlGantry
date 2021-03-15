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
param.N=20;
param.Ts = param.Tf/param.N
param.tolerances=shape.tolerances.state(1:8)*0.5;
param.nx = 8;
param.ny = 8;
param.nu = 2;

load('Crane_NominalParameters.mat');
param.nlobj = nlmpc(param.nx,param.ny,param.nu);
param.nlobj.Model.StateFcn = @(x,u) crane_nl_model_mod(x, u, Tx, Ty ,Tl ,Vx ,Vy ,MR ,M ,m, r, Vl);
param.nlobj.Ts = param.Ts;
param.nlobj.PredictionHorizon = param.N;
param.nlobj.ControlHorizon = param.N;
param.nlobj.Weights.OutputVariables = [10 0 10 0 5 0 5 0];

% Add constraints on inputs
for ct = 1:param.nu
    param.nlobj.MV(ct).Min = -1;
    param.nlobj.MV(ct).Max = 1;
end
% param.nlobj.Optimization.ReplaceStandardCost=true;
param.nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data)myIneqConFunction(X,U,e,data,shape.constraints,r,param);
% param.nlobj.Optimization.CustomCostFcn = @myCostFunction;
param.nloptions = nlmpcmoveopt;
pos = 1; vel = 100; angle = 0 ; angle_vel = 2; 

Q = diag([pos vel pos vel angle angle_vel angle angle_vel]);
%Q = [pos vel pos vel angle angle_vel angle angle_vel];
 
%R = [2 2];
R = diag([1 1]);

J = 0; 
%% Input rate
% nlobj.Weights.ManipulatedVariablesRate = [0 0];
costFun_k = @(x,u) x'*Q*x + u'*R*u + J*(max(0,u(1)*x(2)) + max(0,u(2)*x(3))); 

% nlobj.Weights.OutputVariables = Q;
% nlobj.Weights.ManipulatedVariables = R ; 


nlobj.Optimization.CustomCostFcn = @(x,u,e,data) genCostFun(x,u,e,data,costFun_k);


x0 = [-10;0;-10; 0; 0; 0; 0; 0];  % robot parks at [-10, -10], facing north
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
u=zeros(param.nu,1);
% Create the output array of the appropriate size
persistent it lastMv options

if isempty(it)
    options = nlmpcmoveopt;
    it=0;
    lastMv = zeros(param.nu,1);
end 

% if (it>1)
%     param.nlobj.Optimization.UseSuboptimalSolution=true;
%     param.nlobj.Optimization.SolverOptions.MaxIter=2;
% end 

if (it<param.N-1)
    param.nlobj.PredictionHorizon = param.N-it;
    param.nlobj.ControlHorizon = param.N-it;
    [u,options,info] = nlmpcmove(param.nlobj,x_hat,lastMv,param.targetMod.',[],options);
    conv=info.Iterations
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


function [DRect,clRect,chRect] = getRectConstraints(rect)
%Compute constraint on the cart based on rectangular shape

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
DRect=zeros(2,2);DRect(1,1)=a1;DRect(1,2)=b1;DRect(2,1)=a3;DRect(2,2)=b3;
clRect=[min(c2,c1);min(c4,c3)];
chRect=[max(c1,c2);max(c3,c4)];
end



function J = myCostFunction(X,U,e,data)
% xWork=max(U(1:end-1,data.MVIndex(1)).*X(2:end,2),0);
% yWork=max(U(1:end-1,data.MVIndex(2)).*X(2:end,4),0);[10 0 10 0 5 0 5 0]
Q=[10 0 10 0 5 0 5 0];
R=[1 1];
J=max(0,U(1:end-1,1)*X(2:end)) + max(0,U(1:end-1,1)*X(4));
J=0
end 

function [ costFun ] = genCostFun(x,u,e,data,costFunk)
p = data.PredictionHorizon; 
costFun = [];

for i = 2:p+1
    xk = x(i,:);
    uk = u(i,:);
    costFun = costFunk(xk',uk');
end
end

% costFun_k = @(x,u) x'*Q*x + u'*R*u + J*(max(0,u(1)*x(1)) + max(0,u(2)*x(3))); 

function cineq = myIneqConFunction(X,U,e,data,constraints,r,param)

p = data.PredictionHorizon;
posX = X(2:p+1,1);
posY = X(2:p+1,3);
angleX = X(2:p+1,5);
angleY = X(2:p+1,7);
stepsLeft = size(X,1);

[DRect,clRect,chRect] = getRectConstraints(constraints.rect);

cineq = [
%    Constraints on Cart
     posX*DRect(1,1)+posY*DRect(1,2)-chRect(1);
    -posX*DRect(1,1)-posY*DRect(1,2)+clRect(1);
     posX*DRect(2,1)+posY*DRect(2,2)-chRect(2);
    -posX*DRect(2,1)-posY*DRect(2,2)+clRect(2);
%    Constraints on Load
     DRect(1,1)*(posX+r*angleX) + DRect(1,2)*(posX+r*angleX)-chRect(1);
    -DRect(1,1)*(posX+r*angleX) - DRect(1,2)*(posY+r*angleY)+clRect(1);
     DRect(2,1)*(posX+r*angleX) + DRect(2,2)*(posY+r*angleY)-chRect(2);
    -DRect(2,1)*(posX+r*angleX) - DRect(2,2)*(posY+r*angleY)+clRect(2);
     ];
%  Constraints on cart for elipsis   
 for i = 1:length(constraints.ellipses)
         el=constraints.ellipses{i};
         elCon=-((((posX)-el.xc)/el.a).^2 + (((posY)-el.yc)/el.b).^2) +1.05;
         elConLd=-((((posX+r*angleX)-el.xc)/el.a).^2 + (((posY+r*angleY)-el.yc)/el.b).^2) +1.05;
         cineq = [cineq; elCon; elConLd]; 
 end

for i= 1:min(5,stepsLeft-1)
    xFin=X(p+2-i,:);
    cineq=[ cineq;
            xFin' - param.targetMod - param.tolerances;
           -xFin' + param.targetMod - param.tolerances;
            xFin(1) + r*xFin(5) - param.target(1) - param.tolerances(1);
           -xFin(1) - r*xFin(5) + param.target(1) - param.tolerances(1);
            xFin(3) + r*xFin(7) - param.target(2) - param.tolerances(3);
          - xFin(3) - r*xFin(7) + param.target(2) - param.tolerances(3)
          ];
end

end

 

