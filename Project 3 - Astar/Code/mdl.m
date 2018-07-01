%% function Move down and left
function [Flagdownleft, nNodedownleft] = mdl(cNode)
     x = cNode(1,1);
     y = cNode(1,2);
if x == 0 || y == 0
    Flagdownleft = false;
    nNodedownleft = cNode;
else
    X1 = x-1;
    Y1 = y-1;
    Flagdownleft = true;
    nNodedownleft = [X1, Y1];
end