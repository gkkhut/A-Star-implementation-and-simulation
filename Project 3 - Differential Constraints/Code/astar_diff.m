clear all
close all

%% Defining the whole plot from the given coordinates
figure
% Square 1
xs1= [1.575,1.575,2.375,2.375];
ys1=[7.375,9.375,9.375,7.375];
fill(xs1,ys1,'g');
hold on;
% Square 2
xs2= [2.775,2.775,3.575,3.575];
ys2=[7.375,9.375,9.375,7.375];
fill(xs2,ys2,'b');
hold on;
% Square 3
xs3= [12.05,12.05,13.65,13.65];
ys3=[8.55,9.65,9.65,8.55];
fill(xs3,ys3,'y');
hold on;
% Square 4
xs4= [14.05,14.05,14.85,14.85];
ys4=[4.275,6.275,6.275,4.275];
fill(xs4,ys4,'r');
hold on;
% Square 5
xs5= [14.05,14.05,14.85,14.85];
ys5=[2.275,4.275,4.275,2.275];
fill(xs5,ys5,'b');
hold on;
%% Plotting Tables
% Ploting Table 1
rectangle('Position', [5.5525 3.65 1.6 2.7525],'Curvature',[1 0.5],'FaceColor','blue')
% axis equal
hold on;
% Plotting Table 2
rectangle('Position', [9.3 3.65 1.6 2.7525],'Curvature',[1 0.5],'FaceColor','blue')
hold on;

%% User Input
Flag = false; %true only if all points are in free space 

while ~Flag

title("Select End Point on the Graph")
[r c] = ginput(1); %Graphical Input for goal position
xg = round(r(1),0); %rounding to nearest whole value
yg = round(c(1),0); %rounding to nearest whole value
xs = 1; % x for start Point 
ys = 1; % x for start Point
theta = 1; %theta initialisation
sNode = [xs,ys,theta]; %Start Node
gNode = [xg,yg,theta]; %Goal Node

instart = constraints(xs,ys); %defining constraints using half planes 
ingoal = constraints(xg,yg); %defining constraints using half planes
%% Check whether input lies in the constraints space
if instart
Flag = false; %Flag check
disp('Start point is in the object'); %error display if start point is in object
elseif ingoal
Flag = false;
disp('Goal point is in the object'); %%error display if goal point is in object
elseif (xs<0 || xs>15) || (ys<0 || ys>10)
Flag = false;
disp('Start point is not in the map') %error display
elseif (xg<0 || xg>15) || (yg<0 || yg>=10)
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
cNode = Nodes(:,:,j); %Current node initialisation
clNode(:,:,k) = cNode; %close node initialisation
id = getid(cNode); %get id of the current node
clnInfo(:,:,k) = [id]; %closednode info
drawnow %plot current nodes
plot(cNode(1),cNode(2),'*','color','red')

[Flag, nNode] = m1(cNode); %move 1
if Flag == true %flag check
in = constraints(nNode(1),nNode(2)); %constraints check 
testid = getid(nNode); %get id 
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j); %cost to come
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1; % increament child node
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end

[Flag, nNode] = m2(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end
        
[Flag, nNode] = m3(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end
        
[Flag, nNode] = m4(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end 
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end
        
[Flag, nNode] = m5(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end
[Flag, nNode] = m6(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end

[Flag, nNode] = m7(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end

[Flag, nNode] = m8(cNode);
if Flag == true
in = constraints(nNode(1),nNode(2));
testid = getid(nNode);
% Search if the New Node generated is present in Nodes array or not. 
if in == false
% Save the node if the generated node is not in the obstacle
if (~any(testid == nInfo(1,6,:))) 
ctc = nInfo(:,3,j);
g = ctc + straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
Nodes(:,:,i) = nNode;
nInfo(:,:,i) = [i,j,g,h,f,testid];
i = i+1;
drawnow
plot(nNode(1),nNode(2),'*','color','green')
elseif  (~any(testid == clnInfo(1,1,:)))
k = find(testid == nInfo(1,6,:));
cost = nInfo(1,5,k);
ctc = nInfo(:,3,j);
g = ctc+straightcost;
h = (sqrt(((nNode(1)-gNode(1))^2)+((nNode(2)-gNode(2))^2)+((nNode(3)-gNode(3)^2))));
f = g+h;
if cost>f
nInfo(:,:,k) = [i,j,g,h,f,testid];
end
end
if nNode(1) == gNode(1) && nNode(2) == gNode(2) && nNode(3) == gNode(3)
break
end   
end
end    
%% Finding parent node with minimum cost        
min_cost = [inf,0];
for y = 1:i-1
testid = getid(Nodes(:,:,y));
if (~any(testid == clnInfo(1,1,:)))
cost = nInfo(1,5,y);
if cost < min_cost(1,1)
min_cost = [cost , y];
end
end
end
%Plotting the path
j = min_cost(1,2);
k = k+1;
if j == 10;
break    
k
end
end
q = i-1;
t = 1;
count = 0;
path = [];
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
path(:,:,t) = [u;v];
info = nInfo(1,2,q);
q = info;
count = count+1;
plot(u,v,'.','color','green') %Plot the Path
t=t+1;
end
end