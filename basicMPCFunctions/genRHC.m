function K = genRHC(H,G,m)
% H and G are the cost function matrices
% m is the number of control inputs
% K is the RHC law gain
K=-H\G;
K=K(1:m,:);
end