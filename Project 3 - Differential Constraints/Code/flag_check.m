function Flag = flag_check (nNode)

x = nNode(1); % assign coordinate x
y = nNode(2); % assign coordinate y

length = 15; % row size of map
height = 10; % col size of map

if (x > 0) && (x < length+1) && (y > 0) && (y < height+1)
Flag = true;
elseif (x <= 15) || (y <= 10)
Flag = false;
end
end