%% function Move left
function [Flagleft, nNodeleft] = ml(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 0
    Flagleft = false;
    nNodeleft = [];
else
    X1 = x-1;
    Y1 = y;
    Flagleft = true;
    nNodeleft = [X1, Y1];
end