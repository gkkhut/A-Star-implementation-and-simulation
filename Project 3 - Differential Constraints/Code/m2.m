%% function Move 2
function [Flag, nNode] = m2(cNode)
nNode = cNode;
ul = 100; %Linear velocity
ur = 0; %angular velocity
x_prev = nNode(1);
y_prev = nNode(2);
theta_prev = nNode(3);

Flag = flag_check(nNode); %Flag Check
if Flag
nNode = differential (ul, ur, x_prev,y_prev,theta_prev);
end
end