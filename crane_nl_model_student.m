function dx = crane_nl_model_student(u, x, craneParams)

dx = zeros(10, 1);

% Extract the crane parameters
Tx = craneParams.Tx; % friction coefficient
Ty = craneParams.Ty; % friction coefficient
Tl = craneParams.Tl; % friction coefficient
Vx = craneParams.Vx; % force coefficient
Vy = craneParams.Vy; % force coefficient
Vl = craneParams.Vl; % force coefficient
MR = craneParams.MR; % mass of rail
M  = craneParams.M; % mass of cart
m  = craneParams.m; % mass of pendulum

% This we don't change - after all, all experiments are done on Earth :)
g  = 9.81;

% Extract the states
x1 = x(1); % x
x2 = x(2); % x dot
x3 = x(3); % y
x4 = x(4); % y dot
x5 = x(5); % theta
x6 = x(6); % theta dot
x7 = x(7); % phi
x8 = x(8); % phi dot
x9 = x(9); % l
x10 = x(10); % l dot

% Saturate inputs
u = min( max( u, -1 ), 1 );

% Compute the input forces
fx = Vx.*u(1);
fy = Vy.*u(2);
fl = Vl.*u(3);

% Compute the derivatives
dx(1, 1)  = x2;
dx(2, 1)  = (cos(x5) .* cos(x7) .^ 2 .* sin(x5) .* g .* m + sin(x5) .* (Tl .* x10 - fl) .* cos(x7) - Tx .* x2 + fx) ./ (M + MR);
dx(3, 1)  = x4;
dx(4, 1)  = ((m .* g .* cos(x5) .* cos(x7) + Tl .* x10 - fl) .* sin(x7) - Ty .* x4 + fy) ./ M;
dx(5, 1)  = x6;  
dx(6, 1)  = (-cos(x5) .^ 2 .* cos(x7) .^ 2 .* sin(x5) .* g .* m + (-sin(x5) .* (Tl .* x10 - fl) .* cos(x5) - 0.2e1 .* x6 .* x10 .* (M + MR)) .* cos(x7) + (Tx .* x2 - fx) .* cos(x5) + 0.2e1 .* (M + MR) .* (sin(x7) .* x6 .* x8 .* x9 - sin(x5) .* g ./ 0.2e1)) ./ x9 ./ cos(x7) ./ (M + MR);
dx(7, 1)  = x8;
dx(8, 1)  = (-cos(x5) .* g .* sin(x7) .* m .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 2 + ((-M .* (Tl .* x10 - fl) .* cos(x5) .^ 2 - M .^ 2 .* x6 .^ 2 .* x9 - x6 .^ 2 .* x9 .* MR .* M - MR .* (Tl .* x10 - fl)) .* sin(x7) - (-Ty .* x4 + fy) .* (M + MR)) .* cos(x7) - 0.2e1 .* ((g .* (M + MR) .* cos(x5) ./ 0.2e1 + sin(x5) .* (Tx .* x2 - fx) ./ 0.2e1) .* sin(x7) + x10 .* x8 .* (M + MR)) .* M) ./ x9 ./ M ./ (M + MR);
dx(9, 1)  = x10;
dx(10, 1) = (cos(x5) .* g .* m .^ 2 .* (M .* cos(x5) .^ 2 + MR) .* cos(x7) .^ 3 + m .* (M .* (Tl .* x10 - fl) .* cos(x5) .^ 2 + M .^ 2 .* x6 .^ 2 .* x9 + x6 .^ 2 .* x9 .* MR .* M + MR .* (Tl .* x10 - fl)) .* cos(x7) .^ 2 + (-g .* m .* (M + MR) .* cos(x5) + M .* sin(x5) .* (Tx .* x2 - fx)) .* m .* cos(x7) + (M + MR) .* (-m .* (-Ty .* x4 + fy) .* sin(x7) + (M .* x8 .^ 2 .* x9 - Tl .* x10 + fl) .* m - M .* (Tl .* x10 - fl))) ./ m ./ M ./ (M + MR);
