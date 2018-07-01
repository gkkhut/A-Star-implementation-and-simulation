clear all
close all

%% Defining the whole plot from the given coordinates
rectangle ('Position', [55 67.5 50 45],'FaceColor','blue') %55, 67.5 is the initial point and 50&45  is width and length 
hold on
%Ploting Circle
circle = [165 105 30 30];
rectangle('Position',circle,'Curvature',[1 1],'FaceColor','blue')
axis equal

%Ploting Polygon
pgon = polyshape([145 14; 168 14; 188 51; 165 89; 158 51; 120 55]);
plot(pgon,'FaceColor','blue','FaceAlpha',1)
axis([0 250 0 150])

%% User Input
Flag = false; %true only if all points are in free space 
disp('')
while ~Flag
xstart = 'Enter X for Start Point (0-250:)'; %define start point (x-axis)
xs = input(xstart);
ystart = 'Enter Y for Start Point (0-150:)'; %define start point (y-axis)
ys = input(ystart);

xgoal = 'Enter X for Goal Point (0-250:)'; %define goal point (x-axis)
xg = input(xgoal);
ygoal = 'Enter Y for Start Point (0-150:)'; %define goal point (y-axis)
yg = input(ygoal);

sNode = [xs,ys]; %Start Node
gNode = [xg,yg]; %Goal Node

instart = constraints(xs,ys);
ingoal = constraints(xg,yg); 
%% Check whether input lies in the constraints space
if instart
Flag = false;%status check
disp('Start point is in the object'); %error display
elseif ingoal
Flag = false;
disp('Goal point is in the object'); %error display
elseif (xs<0 || xs>250) || (ys<0 || ys>150)
Flag = false;
disp('Start point is not in the map') %error display
elseif  (xg<0 || xg>250) || (yg<0 || yg>=150)
Flag = false;
disp('Goal point is not in the map') %error display 
else
Flag = true; %status check
end
end

if Flag
Nodes = []; %Nodes
nInfo = []; %Nodes Info

Nodes(:,:,1) = sNode; %Initialise Start Node
id = getid(sNode); 
nInfo(:,:,1) = [1,0,0,0,0,id]; %1st node information
clNode = []; %Closed Node
clNodesInfo = []; %Closed Node Info
    
straightcost = 1; %Straight Cost
diagonalcost = sqrt(2); %Diagonal Cost

i = 2; %Child Node Number
j = 1; %Parent Node Number
k = 1;
%% Generating Nodes using A* Algorithm

while true
% Initialize the parent node in each loop
cNode = Nodes(:,:,j);
clNode(:,:,k) = cNode;
id = getid(cNode);
clnInfo(:,:,k) = [id];
drawnow
plot(cNode(1),cNode(2),'.','color','red')

[Flagleft, nNodeleft] = ml(cNode);
if Flagleft == true
in = constraints(nNodeleft(1),nNodeleft(2));
testid = getid(nNodeleft);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodeleft(1)-gNode(1))^2)+((nNodeleft(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodeleft;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1; % increament child node
drawnow
plot(nNodeleft(1),nNodeleft(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodeleft(1)-gNode(1))^2)+((nNodeleft(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodeleft(1) == gNode(1) && nNodeleft(2) == gNode(2)
break
end   
end
end

[Flagright, nNoderight] = mr(cNode);
if Flagright == true
in = constraints(nNoderight(1),nNoderight(2));
testid = getid(nNoderight);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNoderight(1)-gNode(1))^2)+((nNoderight(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNoderight;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNoderight(1),nNoderight(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNoderight(1)-gNode(1))^2)+((nNoderight(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNoderight(1) == gNode(1) && nNoderight(2) == gNode(2)
break
end   
end
end
        
[Flagup, nNodeup] = mu(cNode);
if Flagup == true
in = constraints(nNodeup(1),nNodeup(2));
testid = getid(nNodeup);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodeup(1)-gNode(1))^2)+((nNodeup(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodeup;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNodeup(1),nNodeup(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodeup(1)-gNode(1))^2)+((nNodeup(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodeup(1) == gNode(1) && nNodeup(2) == gNode(2)
break
end   
end
end
        
[Flagdown, nNodedown] = md(cNode);
if Flagdown == true
in = constraints(nNodedown(1),nNodedown(2));
testid = getid(nNodedown);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodedown(1)-gNode(1))^2)+((nNodedown(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodedown;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNodedown(1),nNodedown(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodedown(1)-gNode(1))^2)+((nNodedown(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodedown(1) == gNode(1) && nNodedown(2) == gNode(2)
break
end   
end
end
        
[Flagdownleft, nNodedownleft] = mdl(cNode);
if Flagdownleft == true
in = constraints(nNodedownleft(1),nNodedownleft(2));
testid = getid(nNodedownleft);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodedownleft(1)-gNode(1))^2)+((nNodedownleft(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodedownleft;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNodedownleft(1),nNodedownleft(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodedownleft(1)-gNode(1))^2)+((nNodedownleft(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodedownleft(1) == gNode(1) && nNodedownleft(2) == gNode(2)
break
end   
end
end
[Flagdownright, nNodedownright] = mdr(cNode);
if Flagdownright == true
in = constraints(nNodedownright(1),nNodedownright(2));
testid = getid(nNodedownright);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodedownright(1)-gNode(1))^2)+((nNodedownright(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodedownright;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNodedownright(1),nNodedownright(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodedownright(1)-gNode(1))^2)+((nNodedownright(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodedownright(1) == gNode(1) && nNodedownright(2) == gNode(2)
break
end   
end
end

[Flagupleft, nNodeupleft] = mul(cNode);
if Flagupleft == true
in = constraints(nNodeupleft(1),nNodeupleft(2));
testid = getid(nNodeupleft);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodeupleft(1)-gNode(1))^2)+((nNodeupleft(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodeupleft;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNodeupleft(1),nNodeupleft(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodeupleft(1)-gNode(1))^2)+((nNodeupleft(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodeupleft(1) == gNode(1) && nNodeupleft(2) == gNode(2)
break
end   
end
end

[Flagupright, nNodeupright] = mur(cNode);
if Flagupright == true
in = constraints(nNodeupright(1),nNodeupright(2));
testid = getid(nNodeupright);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNodeupright(1)-gNode(1))^2)+((nNodeupright(2)-gNode(2))^2)));
f = g+h;
Nodes(:,:,i) = nNodeupright;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNodeupright(1),nNodeupright(2),'.','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNodeupright(1)-gNode(1))^2)+((nNodeupright(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNodeupright(1) == gNode(1) && nNodeupright(2) == gNode(2)
break
end   
end
end    
        
mincost = [inf,0];
for y = 1:i-1
testid = getid(Nodes(:,:,y));
if (~any(testid == clnInfo(1,1,:)))
cost = nInfo(1,5,y);
if cost<mincost(1,1)
mincost = [cost , y];
end
end
end
j = mincost(1,2);
k = k+1;
if rem(k,10) == 0
k
end
end
q = i-1;
count = 0;

txt1 = '\o Start Node'; %Caption - Start node
txt2 = '\o Goal Node'; %Caption - Goal Node
%Plot the start and end point
plot(sNode(1),sNode(2),'s','color','red','markers',10)
plot(gNode(1),gNode(2),'s','color','red','markers',10)
 
text(sNode(1),sNode(2),txt1)
text(gNode(1),gNode(2),txt2)

%Plotting the path
while q ~= 1 
nInfo(:,:,q);
u = Nodes(1,1,q);
v = Nodes(1,2,q); 
info = nInfo(1,2,q);
q = info;
count = count+1;
plot(u,v,'.','color','green') %Plot the Path
end
end