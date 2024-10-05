function [] = boxPlotter(x1, x2, x3)

cla

hold on;
%% Box 3
box_three_width = 5;
box_three_height = 5;
box_three_thickness = 0.1;

box_three_x_1 = (box_three_width/2)*[-1 1 1 -1] + x3;
box_three_x_2 = (box_three_width/2)*[(-1 + box_three_thickness) (1 - box_three_thickness) (1 - box_three_thickness) (-1 + box_three_thickness)] + x3;

box_three_y_1 = (box_three_height)*[0 0 1 1];
box_three_y_2 = (box_three_height)*[(0 + box_three_thickness) (0 + box_three_thickness) (1 - box_three_thickness) (1 - box_three_thickness)];

box_three = polyshape({box_three_x_1, box_three_x_2},{box_three_y_1, box_three_y_2});

temp = sort(box_three_x_2);
xmax = temp(3);
xmin = temp(2);

temp2 = sort(box_three_y_2);
ymax = temp2(3);
ymin = temp2(2);

% Get the boundaries here of xmax and xmin

plot(box_three);

%% Box 2
box_two_width = 1;
box_two_height = (ymax-ymin)/2;

box_two_x = (box_two_width/2)*[-1 1 1 -1] + x2;

if max(box_two_x) > xmax
    box_two_x = box_two_x - (max(box_two_x) - xmax);
elseif min(box_two_x) < xmin
    box_two_x = box_two_x - (min(box_two_x) - xmin);
end

box_two_y = [(ymax-box_two_height) (ymax-box_two_height) ymax ymax];

fill(box_two_x, box_two_y, 'g');


%% Box 1

box_one_width = 2;
box_one_height = ymax - box_two_height - ymin;


box_one_x = (box_one_width/2)*[-1 1 1 -1] + x1;

if max(box_one_x) > xmax
    box_one_x = box_one_x - (max(box_one_x) - xmax);
elseif min(box_one_x) < xmin
    box_one_x = box_one_x - (min(box_one_x) - xmin);
end


box_one_y = [ymin ymin (ymin+box_one_height) (ymin+box_one_height)];%[(ymax-box_two_height) (ymax-box_two_height) ymax ymax]

fill(box_one_x, box_one_y, 'b');



%% Ground

fill([-3 -3 5 5], [0 -.2 -.2 0], 'r')

%% Springs

yval = mean(box_two_y);
fill([xmin, xmin, min(box_two_x), min(box_two_x)], [yval+.01, yval - 0.01, yval - 0.01, yval + 0.01], 'k')

yval = mean(box_one_y);
fill([xmax, xmax, max(box_one_x), max(box_one_x)], [yval+.01, yval - 0.01, yval - 0.01, yval + 0.01], 'k')


hold off


axis([-3 5 -.5 6])

end