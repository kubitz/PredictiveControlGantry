function [ fig ] = plotCourse( course, titleString, dis )
%PLOTCOURSE Summary of this function goes here
%   Detailed explanation goes here

%% Extract the shape in the course
shape = course.shape;


%% Create the figure and add stuff to it
if( dis )
    fig = figure( 'Name', titleString );
else
    fig = figure( 'Name', titleString, 'Visible', 'off' );
end
hold on;

title( titleString );


%% Add the constraint boundaries
fill( shape.constraints.rect(:,1), shape.constraints.rect(:,2), 'white' );

% Save the limits for the rectangle so we can restore them after adding the ellipses
xl = xlim();
yl = ylim();

% Add the ellipses if there are any
if( isfield( shape.constraints, 'ellipses' ) && ~isempty( shape.constraints.ellipses ) )
    for( i=1:1:length( shape.constraints.ellipses ) )
        ellipse = shape.constraints.ellipses{i};

        t = -pi:0.01:pi;
        x = ellipse.xc + ellipse.a * cos( t );
        y = ellipse.yc + ellipse.b * sin( t );
        plot( x, y, 'Color', 'red' );

        fill( [x, ellipse.xc], [y, ellipse.yc], 'red', 'EdgeColor', 'none' );
    end
end
    
% Restore the previous plot limits
xlim( xl );
ylim( yl );


%% Add the starting point
plot( shape.start(:,1), shape.start(:,2), 'bo' );


%% Add the target point (with epsilon size)
rectangle( 'Position', [shape.target(:,1)-shape.eps_t, shape.target(:,2)-shape.eps_t, 2*shape.eps_t, 2*shape.eps_t] )
plot( shape.target(:,1), shape.target(:,2), 'b*' );


%% Mark the outside of the constraints area
set( gca, 'color', 'red' );


yl = ylim();
xl = xlim();

ydiff = diff( yl );
xdiff = diff( xl );

if( ydiff == xdiff )
    % The axis are already equal
    newyl = yl;
    newxl = xl;
elseif( ydiff > xdiff )
    % The y axis is larger than x - expand x
    expand = ( ydiff - xdiff ) / 2;
    
    newxl = xl + [-expand, expand];
    newyl = yl;
else
    % The x axis is larger than y - expand y
    expand = ( xdiff - ydiff ) / 2;
    
    newxl = xl;
    newyl = yl + [-expand, expand];
end

% Add a slight border around the graph
ylim( newyl + [-0.01, 0.01] );
xlim( newxl + [-0.01, 0.01] );

axis square;

end

