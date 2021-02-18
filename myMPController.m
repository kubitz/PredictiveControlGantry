function u = myMPController(r, x_hat, param)
%% Do not delete this line
u=zeros(2,1);
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
    % Shave off top of matrices as the horizon recedes
    DD=reduceMatrix(DD,param.numConstraints,8);
    EE=reduceMatrix(EE,param.numConstraints,2);
    bb=reduceMatrix(bb,param.numConstraints,0);
    Gamma=reduceMatrix(Gamma,8,2);
    Phi=reduceMatrix(Phi,8,0);
end 

if (param.useShrinkingHorizon==true) && (it<param.N)
    N=param.N-it;
    [F,J,L]=genConstraintMatrices(DD,EE,Gamma,Phi,N);
    [H,G] = genCostMatrices(Gamma,Phi,param.Q,param.R,param.P,N);
    [Lchol , ~] = chol(H,'lower');
    Linv = linsolve(Lchol,eye(size(Lchol)),struct('LT',true)); 
    u = genMPController(Linv,G,F,bb,J,L,x_hat,r,2);
    it=it+1;
end 
end % End of myMPController

