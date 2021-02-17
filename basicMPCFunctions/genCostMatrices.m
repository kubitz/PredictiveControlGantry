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