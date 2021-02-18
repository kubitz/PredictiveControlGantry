%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This file will run a complete simulation of the nonlinear crane model
% with the provided controller/state estimator/target generator in the file
% `FunctionTemplate.m`.
%
% Author: Ian McInerney
% Revision: 2021.1
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear variables
%close all

% The percentage perturbation to perform on each crane model parameter
% The default is 10%
perturbSize = 0.1;

% The part of the core coursework that is being worked on
partNum = 1;


%% Load the parameters for the Simulation model
load('Crane_NominalParameters.mat');


%% Create the shape to test on
testCourse = defaultCourse( 0, partNum );



%% Perturb the parameters
% Extract the crane parameters

% The following parameters could all be perturbed:
% Tx friction coefficient in x direction
% Ty friction coefficient in y direction
% Tl friction coefficient in z direction
% Vx force coefficient in x direction
% Vy force coefficient in y direction
% Vl force coefficient in z direction
% MR mass of rail
% M  mass of cart
% m  mass of pendulum
% r  length

rng(15012);

randPerturb = @() ( ( rand()-0.5 ) * perturbSize ) + 1;
perturbFun = @(x) randPerturb() * x;

craneParams.m  = testCourse.perturb.m  * m;
craneParams.M  = testCourse.perturb.M  * M;
craneParams.MR = testCourse.perturb.MR * MR;
craneParams.r  = testCourse.perturb.r  * r;
craneParams.Tl = testCourse.perturb.Tl * Tl;
craneParams.Tx = testCourse.perturb.Tx * Tx;
craneParams.Ty = testCourse.perturb.Ty * Ty;
craneParams.Vl = testCourse.perturb.Vl * Vl;
craneParams.Vx = testCourse.perturb.Vx * Vx;
craneParams.Vy = testCourse.perturb.Vy * Vy;


%% Extract the student functions
extractFunctions( ['FunctionTemplate.m'], 1 );


%% Declare other simulation parameters
tstep = 0.01; % Step size for simulation. Has to be less than minimum allowed Ts
Fs_default = 20; % sampling frequency
Ts_default = 1/Fs_default; % sampling period

x0 = [ testCourse.shape.start(1,1);  % x of cart
       0                             % x dot of cart
       testCourse.shape.start(1,2);  % y of cart
       0;                            % y dot of cart
       0;                            % theta of pendulum
       0;                            % theta dot of pendulum
       0;                            % phi of pendulum
       0;                            % phi dot of pendulum
       craneParams.r;                % r of pendulum (length)
       0;                            % r dot of pendulum
       0];                           % Internal computation variable


%% Call the setup function for the student
tic;
param = mySetup( testCourse.shape );
setupTime = toc;

fprintf('Setup time: %fs\n', setupTime );


%% Extract the user-defined sampling time, if there is one
if( isfield( param, 'Ts' ) )
    Ts = param.Ts;
    Fs = 1/Ts;
else
    Ts = Ts_default;
    Fs = Fs_default;
end

if( ( Ts < 0.01 ) || ( Ts > 1 ) )
    error( 'Ts must be in the interval [0.01, 1]' )
end


%% Set the simulation time
T = 20;


%% Setup the simulation
% create waiting bar
hw = waitbar( 0,'Please wait...' );
warning( 'on' );

% Initial conditions
x    = x0';
y    = x0(1:8,:);
u    = [0; 0];
ind  = 1;

% Setup variables to hold the results
time    = 0;
inputs  = [u; 0]';
states  = x0';

% Variable to hold optimization time
numSamples  = T/Ts;
allContTime = [0];

% ODE Options
odeOpts = odeset( 'RelTol', 1e-3, 'MaxStep', 0.01 );


%% Iterate for the simulation time
for t=0:Ts:T
    ind = ind+1;
    
    waitbar( t/T, hw, 'Please wait...' );
    tic;

    % Call the state estimator
    x_hat = myStateEstimator( u(1:2,:), y, param );
    
    % Call the target generator
    ref = myTargetGenerator( x_hat, param );
    
    % Call the controller function
    u = myMPController( ref, x_hat, param );
    u = [u; 0]; % we won't use the third input, which is in the Z axis (i.e. the third input increases/decreases the length of the pendulum).
    
    contTime = toc;

    % Setup the simulation model
    mod = @(t, state) crane_nl_model( u, state, craneParams );
    
    % Simulate
    [tt, x] = ode23t( mod, [t:tstep:t + Ts], x(end,:), odeOpts );
    
    % The output is simply all the states except the Z axis portion
    y = x(end,1:8)';

    % Keep the state variables around for analysis
    % We ignore the first entry since it is the same as the entry from the
    % last loop
    time    = [time;    tt(2:end, end)];
    states  = [states;  x(2:end, :)];
    inputs  = [inputs;  u'.*ones( size( tt, 1 ) - 1, 1 ) ];
    
    % Save the computation time for analysis
    allContTime = [allContTime; contTime.*ones( size( tt, 1 ) - 1, 1 )];
end

close(hw);


%% Visualize the Course
% allContTime
[~, ~, ~, text] = analyzeCourse( [], time, states, inputs, allContTime, testCourse, r, 1 );
fprintf(text)
