function [] = boxPlotter(x1, x2, x3)

cla

hold on;
%% Box 3
box_three_width = 2;
box_three_height = 2;
box_three_thickness = 0.1;

box_three_x_1 = (box_three_width/2)*[-1 1 1 -1] + x1;
box_three_x_2 = (box_three_width/2)*[(-1 + box_three_thickness) (1 - box_three_thickness) (1 - box_three_thickness) (-1 + box_three_thickness)] + x1;

box_three_y_1 = (box_three_height/2)*[0 0 1 1];
box_three_y_2 = (box_three_height/2)*[(0 + box_three_thickness) (0 + box_three_thickness) (1 - box_three_thickness) (1 - box_three_thickness)];

box_three = polyshape({box_three_x_1, box_three_x_2},{box_three_y_1, box_three_y_2});

temp = sort(box_three_x_2);
xmax = temp(3);
xmin = temp(2);

% Get the boundaries here of xmax and xmin

plot(box_three);

%% Box 2
box_two_width = 1;
box_two_height = .75;


box_two_x = (box_two_width/2)*[-1 1 1 -1] + x2;

if max(box_two_x) > xmax
    box_two_x = box_two_x - (max(box_two_x) - xmax);
elseif min(box_two_x) < xmin
    box_two_x = box_two_x - (min(box_two_x) - xmin);
end


box_two_y = (box_two_height/2)*[0 0 1 1] + box_three_thickness;

fill(box_two_x, box_two_y, 'b');


%% Box 1
box_one_width = xmax-xmin - box_two_width;
box_one_height = box_three_height - 4*box_three_thickness- box_two_height;

box_one_x = (box_one_width/2)*[-1 1 1 -1] + x3;

if max(box_one_x) > xmax
    box_one_x = box_one_x - (max(box_one_x) - xmax);
elseif min(box_one_x) < xmin
    box_one_x = box_one_x - (min(box_one_x) - xmin);
end

box_one_y = (box_one_height/2)*[0 0 1 1] + max(box_two_y);

fill(box_one_x, box_one_y, 'g');


%% Ground

fill([-3 -3 3 3], [0 -.2 -.2 0], 'r')

%% Springs

yval = mean(box_one_y);
fill([xmin, xmin, min(box_one_x), min(box_one_x)], [yval+.01, yval - 0.01, yval - 0.01, yval + 0.01], 'k')

yval = mean(box_two_y);
fill([xmax, xmax, max(box_two_x), max(box_two_x)], [yval+.01, yval - 0.01, yval - 0.01, yval + 0.01], 'k')


hold off


axis([-3 3 -.5 1.5])

end