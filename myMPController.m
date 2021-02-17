function u = myMPController(r, x_hat, param)
%% Do not delete this line
% Create the output array of the appropriate size
u = zeros(2,1);
u = genMPController(param.Linv,param.G,param.F,param.bb,param.J,param.L,x_hat,r,2);
end % End of myMPController

