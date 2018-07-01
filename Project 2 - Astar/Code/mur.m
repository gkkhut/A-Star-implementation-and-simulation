%% function Move up and right
function [Flagupright, nNodeupright] = mur(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 150 || y == 250
    Flagupright = false;
    nNodeupright = [];
else
    X1 = x+1;
    Y1 = y+1;
    Flagupright = true;
    nNodeupright = [X1, Y1];
end