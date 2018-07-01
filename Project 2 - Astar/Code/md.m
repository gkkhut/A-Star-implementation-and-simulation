%% function Move down
function [Flagdown, nNodedown] = md(cNode)
     x = cNode(1);
     y = cNode(2);
if x == 0
    Flagdown = false;
    nNodedown = [];
else
    X1 = x;
    Y1 = y-1;
    Flagdown = true;
    nNodedown = [X1, Y1];
end