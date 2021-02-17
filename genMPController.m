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

