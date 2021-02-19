function [DRect,clRect,chRect] = getRectConstraintsLoad(rect,length)
A = rect(1,:);
B = rect(2,:);
C = rect(3,:);
D = rect(4,:);
% First two parallel edges - computed in same direction
[a1,b1,c1] = getLineParamsStd(A,B);
[a2,b2,c2] = getLineParamsStd(D,C);
% Second set of parallel edges - computed in same direction
[a3,b3,c3] = getLineParamsStd(B,C);
[a4,b4,c4] = getLineParamsStd(A,D);
% Compute D matrix and upper/lower bounds
DRect=zeros(2,8);DRect(1,1)=a1;DRect(1,3)=b1;DRect(2,1)=a3;DRect(2,3)=b3;
DRect(1,5)=a1*length;DRect(1,7)=b1*length;DRect(2,5)=a3*length;DRect(2,7)=b3*length;
clRect=[min(c2,c1);min(c4,c3)];
chRect=[max(c1,c2);max(c3,c4)];

end

