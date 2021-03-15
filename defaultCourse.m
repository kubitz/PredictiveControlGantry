function [ course ] = defaultCourse( dis, coursenum )
%DEFAULTCOURSE Generate the default course for the core coursework
if coursenum == 0
    % The shape for the first part
    constraints.rect = [0.00, 0.05;
                        0.45, 0.50;
                        0.50, 0.45;
                        0.05, 0.00];
                    
    constraints.ellipses = {};
    
    penalties = [-1, -2, -1, -1];
    
    start  = [0.05, 0.05];
    target = [0.4, 0.4];
    
    Wmax = 1;
    Tf   = 5;
elseif coursenum == 1
    % The shape for the first part
%     constraints.rect = [0.00, 0.05;
%                         0.45, 0.50;
%                         0.50, 0.45;
%                         0.05, 0.00];
                    
    constraints.ellipses = {};
    
    penalties = [-1, -2, -1, -1];
    
%     start  = [0.05, 0.05];
%     target = [0.4, 0.4];
    constraints.rect=[-0.0819,-0.493;0.0917,0.492;0.0819,0.493;-0.0917,-0.492];
    start= [0.0917481275985262, 0.49153563561776936];
    target= [-0.0819,-0.493];    
    Wmax = 1;
    Tf   = 3;

elseif coursenum == 2
    % The shape for the second part
    constraints.rect = [0.00, 0.05;
                        0.25, 0.30;
                        0.50, 0.05;
                        0.25, -0.20];
    ellipse.a  = 0.3;
    ellipse.b  = 0.3;
    ellipse.xc = 0.25;
    ellipse.yc = -0.23;
    constraints.ellipses{1} = ellipse;
    % Define the new ellipse
    ellipse.a  = 0.3;
    ellipse.b  = 0.17;
    ellipse.xc = 0.25;
    ellipse.yc = 0.35;
    constraints.ellipses{2} = ellipse;

    ellipse.a  = 0.01;
    ellipse.b  = 0.01;
    ellipse.xc = 0.1;
    ellipse.yc = 0.08;
    constraints.ellipses{3} = ellipse;
    
    
    penalties = [-1, -1, -1, -1, -1, -1, -1];
    
    start  = [0.05, 0.05];
    target = [0.45, 0.05];
    
    Wmax = 1;
    Tf   = 5;

    
elseif coursenum == 3
%     The shape for the second part
    constraints.rect = [0.00, 0.05;
                        0.25, 0.30;
                        0.50, 0.05;
                        0.25, -0.20];
    penalties = [-1, -2, -1, -1];
    constraints.ellipses = {};
    start= [0.25, 0.30];
    target= [0.5, 0.05];
    Wmax = 1;
    Tf   = 5;

elseif coursenum == 4
    % The shape for the first part
    constraints.rect = [0.00, 0.01;
                        0.49, 0.50;
                        0.50, 0.49;
                        0.01, 0.00];
                   
    constraints.ellipses = {};
   
    penalties = [-1, -2, -1, -1];
   
    start  = [0.5, 0.49];
    target = [0.0, 0.01];
   
    Wmax = 1;
    Tf   = 2.75;
elseif coursenum == 5
    constraints.rect=[-0.75,-0.75;0.75,-0.75;0.75,0.75;-0.75,0.75]

    ellipse0.xc=0.0;
    ellipse0.yc=0.0;
    ellipse0.a=0.25;
    ellipse0.b=0.25;

    ellipse1.xc=0.2;
    ellipse1.yc=0.2;
    ellipse1.a=0.05;
    ellipse1.b=0.75;

    ellipse2.xc=-0.2;
    ellipse2.yc=-0.7;
    ellipse2.a=0.05;
    ellipse2.b=0.35;

    ellipse3.xc=0.0;
    ellipse3.yc=0.0;
    ellipse3.a=0.6;
    ellipse3.b=0.05;
    constraints.ellipses={ellipse0,ellipse1,ellipse2,ellipse3};
    penalties = [-1, -1, -1, -1, -1, -1, -1, -1];
    start=[0, 0.6];
    target=[0.5, 0.5];
    Wmax = 0.2;
    Tf   = 5;
end


% ((posX-el.xc)/el.a).^2 + ((posY-el.yc)/el.b).^2 -1 


shape.constraints = constraints;

tolerances.state = [ 0.02;   % Cart x position
                     0.02;   % Cart y position
                     0.02;   % Cart x velocity
                     0.02;   % Cart y velocity
                     2*pi;   % Theta position
                     0.02;   % Theta velocity
                     2*pi;   % Phi position
                     0.02;   % Phi velocity
                     0.02;   % Payload x position
                     0.02 ]; % Payload y position
tolerances.input = [ 0.02;   % X input
                     0.02 ]; % Y input

shape.tolerances  = tolerances;
shape.eps_r       = 0.02;
shape.eps_t       = 0.02;
shape.start       = start;
shape.target      = target;
shape.Wmax        = Wmax;
shape.Tf          = Tf;

% The default shape is always considered problem 1
course.prob      = 1;
course.shape     = shape;
course.penalties = penalties;
course.completed = 0;

course.perturbSize = 0.1;

% Create the perturbations
rng(15012);
randPerturb = @() ( ( rand()-0.5 ) * course.perturbSize ) + 1;

course.perturb.m  = randPerturb();
course.perturb.M  = randPerturb();
course.perturb.MR = randPerturb();
course.perturb.r  = randPerturb();
course.perturb.Tl = randPerturb();
course.perturb.Tx = randPerturb();
course.perturb.Ty = randPerturb();
course.perturb.Vl = randPerturb();
course.perturb.Vx = randPerturb();
course.perturb.Vy = randPerturb();

end
