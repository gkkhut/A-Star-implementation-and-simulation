%% function Move up and left
function [Flagupleft, nNodeupleft] = mul(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 150 || y == 0
    Flagupleft = false;
    nNodeupleft = [];
else
    X1 = x-1;
    Y1 = y+1;
    Flagupleft = true;
    nNodeupleft = [X1, Y1];
end