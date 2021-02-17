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

