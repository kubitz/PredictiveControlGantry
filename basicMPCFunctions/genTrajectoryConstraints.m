function [DD,EE,bb]=genTrajectoryConstraints(Dt,Et,bt,N)
DD=kron(eye(N),Dt);
EE=kron(eye(N),Et);
bb=kron(ones(N,1),bt);
end