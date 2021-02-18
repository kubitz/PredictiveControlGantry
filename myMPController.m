function u = myMPController(r, x_hat, param)
%% Do not delete this line
u=zeros(param.numInputs,1);
% Create the output array of the appropriate size
persistent it EE DD Gamma Phi bb
if isempty(it)
    it=0;
    EE=param.EE;
    DD=param.DD;
    Gamma=param.Gamma;
    Phi=param.Phi;
    bb=param.bb;
    F=param.F;
    J=param.J;
    L=param.L;
    H=param.H;
    G=param.G;
else
    % Shave off matrices as the horizon shrinks
    DD=reduceMatrix(DD,param.numConstraints,param.numStates);
    EE=reduceMatrix(EE,param.numConstraints,param.numInputs);
    bb=reduceMatrix(bb,param.numConstraints,0);
    Gamma=reduceMatrix(Gamma,param.numStates,param.numInputs);
    Phi=reduceMatrix(Phi,param.numStates,0);
end 

if (param.useShrinkingHorizon==true) && (it<param.N)
    N=param.N-it;
    [F,J,L]=genConstraintMatrices(DD,EE,Gamma,Phi,N);
    [H,G] = genCostMatrices(Gamma,Phi,param.Q,param.R,param.P,N);
    u = genMPController(H,G,F,bb,J,L,x_hat,r,2);
    it=it+1;
end 
end % End of myMPController

