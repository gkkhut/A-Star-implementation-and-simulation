%% function Move 5
function [Flag, nNode] = m5(cNode)
nNode = cNode;
ul = 0; %Linear velocity
ur = 50; %angular velocity
x_prev = nNode(1);
y_prev = nNode(2);
theta_prev = nNode(3);

Flag = flag_check(nNode); %Flag Check
if Flag
nNode = differential (ul, ur, x_prev,y_prev,theta_prev);
end
end