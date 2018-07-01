%% function Move up
function [Flagup, nNodeup] = mu(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 150
    Flagup = false;
    nNodeup = [];
else
    X1 = x;
    Y1 = y+1;
    Flagup = true;
    nNodeup = [X1, Y1];
end