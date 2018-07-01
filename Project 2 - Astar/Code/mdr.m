%% function Move down and right
function [Flagdownright, nNodedownright] = mdr(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 0 || y == 250
    Flagdownright = false;
    nNodedownright = [];
else
    X1 = x+1;
    Y1 = y-1;
    Flagdownright = true;
    nNodedownright = [X1, Y1];
end