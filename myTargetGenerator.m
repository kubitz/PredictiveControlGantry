function r = myTargetGenerator(x_hat, param)
%% Modify the following function for your target generation

% Create the output array of the appropriate size. This vector is passed
% into myMPController, so its size must match what is expected by the code
% there.
r = zeros(8,1);

% Make the crane go to (xTar, yTar)
r(1,1) = param.xTar;
r(3,1) = param.yTar;

end % End of myTargetGenerator

