function out = differential(ul,ur, x_prev,y_prev,theta_prev)
%% 10 hz data generation gives smoother output
time = 10;

scaling = 2;
%% robot dimension in m
r = 0.035;
L = 0.23;

%% Converting rpm to radian per second
ul = (ul/60)*2*pi;
ur = (ur/60)*2*pi;
ul = ul/scaling;
ur = ur/scaling;
thetadot = ((r/L)*(ur-ul));
theta = thetadot*time + theta_prev;
xdot = ((r/2)*(ur+ul))*cos(theta);
ydot = ((r/2)*(ur+ul))*sin(theta);
x = xdot*time + x_prev;
y = ydot*time + y_prev;

out = [x,y,theta];
end