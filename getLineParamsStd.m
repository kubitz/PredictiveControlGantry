function [A,B,C] = getLineParamsStd(pointA,pointB)
% Get line parameters in standard form Ax+By=C
A=pointA(2)-pointB(2);
B=pointB(1)-pointA(1);
C=A*pointA(1)+B*pointA(2);
end
