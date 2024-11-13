% Pendulum animation

clc; clear all; close all; format compact;

t = 0:.01:10;
theta = sin(t);

L = 1;

% Interpolate frames for animation'

%{

    Want to operate the animation at the same frame rate as reality, so
    that the animation doesn't look like it's in slow motion

%}

FPS = 30;
t_animate = 0:(1/FPS):t(end);
theta_animate = interp1(t, theta, t_animate);


figure();

p = 0:.01:2*pi;
xball = .1*cos(p);
yball = .1*sin(p);

% Pumpkin ellipses
xpump1 = .2*cos(p) - .08;
ypump1 = .3*sin(p);
xpump2 = .2*cos(p) + .08;
ypump2 = .3*sin(p);
xeye1 = .5*[.4 .5 .3] -.1;
yeye1 = .5*[.4 .5 .5];

xeye2 = -.5*[.4 .5 .3]  + .1;
yeye2 = .5*[.4 .5 .5];

v1 = [xpump1; ypump1];
v2 = [xpump2; ypump2];

v3 = [xeye1; yeye1];

v4 = [xeye2; yeye2];


for i = 1:length(theta_animate)
    R = [cos(theta_animate(i)), -sin(theta_animate(i)); sin(theta_animate(i)), cos(theta_animate(i))];

    v1r = R*v1;

    xpump1 = v1r(1,:);
    ypump1 = v1r(2,:);

    v2r = R*v2;

    xpump2 = v2r(1,:);
    ypump2 = v2r(2,:);

    v3r = R*v3;

    xeye1 = v3r(1,:);
    yeye1 = v3r(2,:);

    v4r = R*v4;

    xeye2 = v4r(1,:);
    yeye2 = v4r(2,:);


   

    hold on
    h = plot([0 L*sin(theta_animate(i))], [0 -L*cos(theta_animate(i))], '-k');
    h.Color = [1 .5 0]; % Orange
    h.LineWidth = 3;
    h1 = fill((xpump1 + L*sin(theta_animate(i))), (ypump1 + -L*cos(theta_animate(i))), [1 .5 0]);
    h2 = fill((xpump2 + L*sin(theta_animate(i))), (ypump2 + -L*cos(theta_animate(i))), [1 .5 0]);

    h3 = fill((xeye1 + L*sin(theta_animate(i))), (yeye1 + -L*cos(theta_animate(i))), [1 .5 0]);
    h4 = fill((xeye2 + L*sin(theta_animate(i))), (yeye2 + -L*cos(theta_animate(i))), [1 .5 0]);

   
    h1.EdgeColor = "none";
    h2.EdgeColor = "none";

    axis equal
    axis([-2*L 2*L -2*L 2*L])
    drawnow;   
    pause(1/FPS);
    delete(h3);
    delete(h4);
    delete(h2);
    delete(h1);
    delete(h);
    hold off
    
end





