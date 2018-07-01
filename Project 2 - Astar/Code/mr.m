%% function Move right
function [Flagright, nNoderight] = mr(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 250
    Flagright = false;
    nNoderight = [];
else
    X1 = x+1;
    Y1 = y;
    Flagright = true;
    nNoderight = [X1, Y1];
end