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
param.Ts = 0.05;
param.N=ceil(param.Tf/param.Ts);
param.settlingPeriod=1;
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
param.nlobj.Weights.OutputVariables = [1 5 1 5 0 0 0 0];
% param.nlobj.Weights.OutputVariables = [1 0 1 0 0 0 0 0];

param.nlobj.Weights.ManipulatedVariables = [5 5];
nlobj.Optimization.UseSuboptimalSolution = true;

nlobj.Optimization.SolverOptions.FunctionTolerance = 0.001;
nlobj.Optimization.SolverOptions.StepTolerance = 0.001;
nlobj.Optimization.SolverOptions.MaxIter = 35;
nlobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;


% Add constraints on inputs
for ct = 1:param.nu
    param.nlobj.MV(ct).Min = -1;
    param.nlobj.MV(ct).Max = 1;
end
% param.nlobj.Optimization.ReplaceStandardCost=true;
param.nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data)myIneqConFunction(X,U,e,data,shape.constraints,r,param);
param.nloptions = nlmpcmoveopt;

param.pathGuess = getPathGuess(shape, 250, 1.1, param.N);



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
    firstStateGuess=zeros(param.N,8);
    for i = 1:param.N
        firstStateGuess(i,1)=param.pathGuess(i,1);
        firstStateGuess(i,3)=param.pathGuess(i,2);
    end
    options.X0=firstStateGuess;
    param.nlobj.Optimization.RunAsLinearMPC="TimeVarying"
    [u,options,info] = nlmpcmove(param.nlobj,x_hat,lastMv,param.targetMod.',[],options);
%     param.nlobj.Optimization.RunAsLinearMPC="Off";
%     [u,options,info] = nlmpcmove(param.nlobj,x_hat,lastMv,param.targetMod.',[],options);
else 
    param.nlobj.Optimization.RunAsLinearMPC="TimeVarying";
end 
% if (it>1)
%     param.nlobj.Optimization.UseSuboptimalSolution=true;
%     param.nlobj.Optimization.SolverOptions.MaxIter=2;
% end 

if (it<param.N-2)
    param.nlobj.PredictionHorizon = param.N-it;
    param.nlobj.ControlHorizon = param.N-it;
    [u,options,info] = nlmpcmove(param.nlobj,x_hat,lastMv,param.targetMod.',[],options);
    conv=info.Iterations
    it=it+1;
    lastMv=u
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
     posX*DRect(1,1)+posY*DRect(1,2)-chRect(1)-e;
    -posX*DRect(1,1)-posY*DRect(1,2)+clRect(1)-e;
     posX*DRect(2,1)+posY*DRect(2,2)-chRect(2)-e;
    -posX*DRect(2,1)-posY*DRect(2,2)+clRect(2)-e;
%    Constraints on Load
     DRect(1,1)*(posX+r*angleX) + DRect(1,2)*(posX+r*angleX)-chRect(1)-e;
    -DRect(1,1)*(posX+r*angleX) - DRect(1,2)*(posY+r*angleY)+clRect(1)-e;
     DRect(2,1)*(posX+r*angleX) + DRect(2,2)*(posY+r*angleY)-chRect(2)-e;
    -DRect(2,1)*(posX+r*angleX) - DRect(2,2)*(posY+r*angleY)+clRect(2)-e;
     ];
% %  Constraints on cart for elipsis   
 for i = 1:length(constraints.ellipses)
         el=constraints.ellipses{i};
         elCon=-((((posX)-el.xc)/el.a).^2 + (((posY)-el.yc)/el.b).^2) +1.01+e;
         elConLd=-((((posX+r*angleX)-el.xc)/el.a).^2 + (((posY+r*angleY)-el.yc)/el.b).^2) +1.01+e;
         cineq = [cineq; elCon; elConLd]; 
 end
% 
for i= 1:min(10,stepsLeft-1)
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

function map = getMap(constraints,res,e)
    map = false(res,res);
    isOut = @(x,y) isConstraint(x,y,constraints,e);
    for x = 1:size(map,1)
        for y = 1:size(map,2)
           xt =  1 - (2/res)*x;
           yt =  1 -(2/res)*y;
           map(x,y) = isOut(xt,yt);
        end
    end
end

function c = isConstraint(x,y,constraints,e)
 [DRect,clRect,chRect] = getRectConstraints(constraints.rect);
 
 c1=(x*DRect(1,1)+y*DRect(1,2)-chRect(1))>0;
 c2=(-x*DRect(1,1)-y*DRect(1,2)+clRect(1))>0;
 c3=(x*DRect(2,1)+y*DRect(2,2)-chRect(2))>0;
 c4=(-x*DRect(2,1)-y*DRect(2,2)+clRect(2))>0;
 c= c1||c2||c3||c4;

 for i = 1:length(constraints.ellipses)
         el=constraints.ellipses{i};
         c = c || (-(((x-el.xc)/el.a).^2 + ((y-el.yc)/el.b).^2) +1)>0; 
 end 
end

function pt = resamplePath(t,px,py)
% resamplePath: interpolate points along a curve in 2 or more dimensions
% modified version of interparc by John D'Errico
t = t(:);
if (numel(t) == 1) && (t > 1) && (rem(t,1) == 0)
  t = linspace(0,1,t)';
elseif any(t < 0) || any(t > 1)
  error('ARCLENGTH:impropert', ...
    'All elements of t must be 0 <= t <= 1')
end
nt = numel(t);
px = px(:);
py = py(:);
n = numel(px);
if ~isvector(px) || ~isvector(py) || (length(py) ~= n)
  error('ARCLENGTH:improperpxorpy', ...
    'px and py must be vectors of the same length')
elseif n < 2
  error('ARCLENGTH:improperpxorpy', ...
    'px and py must be vectors of length at least 2')
end
pxy = [px,py];
ndim = 2;
pt = NaN(nt,ndim);
chordlen = sqrt(sum(diff(pxy,[],1).^2,2));
chordlen = chordlen/sum(chordlen);
cumarc = [0;cumsum(chordlen)];
[junk,tbins] = histc(t,cumarc); 
tbins((tbins <= 0) | (t <= 0)) = 1;
tbins((tbins >= n) | (t >= 1)) = n - 1;
s = (t - cumarc(tbins))./chordlen(tbins);
pt = pxy(tbins,:) + (pxy(tbins+1,:) - pxy(tbins,:)).*repmat(s,1,ndim);
return
end 


function pathGuess = getPathGuess(shape, res, e, predictionHorizon)
    %  Constraints is the constraint datastructure in shape 
    %  res is the resolution for the discretization of the constraints (higher is better but longer processing time)
    %  e is by how much the constraint are enlarged [1 2]
    %  prediction horizon is how many steps the guess has
    constraints=shape.constraints;
    draw = false;
    MAP = int8(getMap(constraints,res,e));
    %Start Positions
    StartY=floor((1-shape.start(1))*(res/2));
    StartX=floor((1-shape.start(2))*(res/2));
    EndX=floor((1-shape.target(1))*(res/2));
    EndY=floor((1-shape.target(2))*(res/2));
    GoalRegister=int8(zeros(res,res));
    GoalRegister(EndX,EndY)=1;
    Connecting_Distance=8; 
    OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance);
    % Convert discrete map to continous values
    pathGuess = zeros(size(OptimalPath));
    for i=1:size(pathGuess,1)
        pathGuess(i,1) = 1-OptimalPath(i,1)*(2/res);
        pathGuess(i,2) = 1-OptimalPath(i,2)*(2/res);
    end
    % Resample map to prediction horizon
    pathGuess = resamplePath(predictionHorizon,pathGuess(:,1),pathGuess(:,2));   
    if draw
        path = resamplePath(predictionHorizon,OptimalPath(:,1),OptimalPath(:,2));   
        MAP = int8(getMap(constraints,res,e));
        figure(10)
        imagesc((MAP))
        colormap(flipud(gray));
        hold on
        plot(path(1,2),path(1,1),'o','color','k')
        plot(path(end,2),path(end,1),'o','color','b')
        plot(path(:,2),path(:,1),'r')
        legend('Target','Goal','Path')
    end
    %little dirty fix for my poor handling of dimensions
    pathGuess=flip(pathGuess);
    pathGuess(1,:)=shape.start;
    pathGuess(end,:)=shape.target;
end

function OptimalPath = ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance)
%Version 1.0
% By Einar Ueland 2nd of May, 2016

%FINDING ASTAR PATH IN AN OCCUPANCY GRID


%nNeighboor=3;
% Preallocation of Matrices
[Height,Width]=size(MAP); %Height and width of matrix
GScore=zeros(Height,Width);           %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width));     %Matrix keeping track of F-scores (only open list) 
Hn=single(zeros(Height,Width));       %Heuristic matrix
OpenMAT=int8(zeros(Height,Width));    %Matrix keeping of open grid cells
ClosedMAT=int8(zeros(Height,Width));  %Matrix keeping track of closed grid cells
ClosedMAT(MAP==1)=1;                  %Adding object-cells to closed matrix
ParentX=int16(zeros(Height,Width));   %Matrix keeping track of X position of parent
ParentY=int16(zeros(Height,Width));   %Matrix keeping track of Y position of parent


%%% Setting up matrices representing neighboors to be investigated
NeighboorCheck=ones(2*Connecting_Distance+1);
Dummy=2*Connecting_Distance+2;
Mid=Connecting_Distance+1;
for i=1:Connecting_Distance-1
NeighboorCheck(i,i)=0;
NeighboorCheck(Dummy-i,i)=0;
NeighboorCheck(i,Dummy-i)=0;
NeighboorCheck(Dummy-i,Dummy-i)=0;
NeighboorCheck(Mid,i)=0;
NeighboorCheck(Mid,Dummy-i)=0;
NeighboorCheck(i,Mid)=0;
NeighboorCheck(Dummy-i,Mid)=0;
end
NeighboorCheck(Mid,Mid)=0;

[row, col]=find(NeighboorCheck==1);
Neighboors=[row col]-(Connecting_Distance+1);
N_Neighboors=size(col,1);
%%% End of setting up matrices representing neighboors to be investigated


%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
[col, row]=find(GoalRegister==1);
RegisteredGoals=[row col];
Nodesfound=size(RegisteredGoals,1);

for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        if MAP(k,j)==0
            Mat=RegisteredGoals-(repmat([j k],(Nodesfound),1));
            Distance=(min(sqrt(sum(abs(Mat).^2,2))));
            Hn(k,j)=Distance;
        end
    end
end
%End of creating Heuristic-matrix. 

%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.

%Initializign start node with FValue and opening first node.
FScore(StartY,StartX)=Hn(StartY,StartX);         
OpenMAT(StartY,StartX)=1;   


while 1==1 %Code will break when path found or when no path exist
    MINopenFSCORE=min(min(FScore));
    if MINopenFSCORE==inf;
    %Failure!
    OptimalPath=[inf];
    RECONSTRUCTPATH=0;
     break
    end
    [CurrentY,CurrentX]=find(FScore==MINopenFSCORE);
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);

    if GoalRegister(CurrentY,CurrentX)==1
    %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
  %Remobing node from OpenList to ClosedList  
    OpenMAT(CurrentY,CurrentX)=0;
    FScore(CurrentY,CurrentX)=inf;
    ClosedMAT(CurrentY,CurrentX)=1;
    for p=1:N_Neighboors
        i=Neighboors(p,1); %Y
        j=Neighboors(p,2); %X
        if CurrentY+i<1||CurrentY+i>Height||CurrentX+j<1||CurrentX+j>Width
            continue
        end
        Flag=1;
        if(ClosedMAT(CurrentY+i,CurrentX+j)==0) %Neiboor is open;
            if (abs(i)>1||abs(j)>1);   
                % Need to check that the path does not pass an object
                JumpCells=2*max(abs(i),abs(j))-1;
                for K=1:JumpCells;
                    YPOS=round(K*i/JumpCells);
                    XPOS=round(K*j/JumpCells);
            
                    if (MAP(CurrentY+YPOS,CurrentX+XPOS)==1)
                        Flag=0;
                    end
                end
            end
             %End of  checking that the path does not pass an object

            if Flag==1;           
                tentative_gScore = GScore(CurrentY,CurrentX) + sqrt(i^2+j^2);
                if OpenMAT(CurrentY+i,CurrentX+j)==0
                    OpenMAT(CurrentY+i,CurrentX+j)=1;                    
                elseif tentative_gScore >= GScore(CurrentY+i,CurrentX+j)
                    continue
                end
                ParentX(CurrentY+i,CurrentX+j)=CurrentX;
                ParentY(CurrentY+i,CurrentX+j)=CurrentY;
                GScore(CurrentY+i,CurrentX+j)=tentative_gScore;
                FScore(CurrentY+i,CurrentX+j)= tentative_gScore+Hn(CurrentY+i,CurrentX+j);
            end
        end
    end
end

k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentY CurrentX];
    while RECONSTRUCTPATH
        CurrentXDummy=ParentX(CurrentY,CurrentX);
        CurrentY=ParentY(CurrentY,CurrentX);
        CurrentX=CurrentXDummy;
        OptimalPath(k,:)=[CurrentY CurrentX];
        k=k+1;
        if (((CurrentX== StartX)) &&(CurrentY==StartY))
            break
        end
    end
end


end

