function [F,J,L] = genConstraintMatrices(DD,EE,Gamma,Phi,N)
sx = size(DD,2) / N;
shift = [zeros(sx, sx*(N-1));eye(sx*(N-1))]*[eye(sx*(N-1)), zeros(sx*(N-1),sx)];
F = DD*shift*Gamma + EE;
J = -DD*(shift*Phi + [eye(sx);zeros(sx*(N-1), sx)]);
L =  - DD * kron(ones(N,1), eye(sx)) - J;
end

