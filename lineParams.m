function [a,c] = lineParams(pointA, pointB)
    % given two points, returns the parameters of the line ax+y=c
    dy = pointB(2)-pointA(2);
    dx = pointB(1)-pointA(1);
    a = - dy/dx; % y + a * x = c
    c = pointA(2) + a * pointA(1);
end

