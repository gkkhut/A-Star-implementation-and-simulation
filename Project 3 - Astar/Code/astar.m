clear all
close all

%% Setting up the communication between matlab and vrep
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
 
if (clientID>-1)
disp('Connected to remote API server');

%% Defining the RRL plot from the given coordinates
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
%Taking goal inputs from graphical inputs  
[r c] = ginput(1); %initialising row and column for inputs
xg = round(r(1),0); %rounding x of goal point
yg = round(c(1),0); %rounding y of goal point
xs = 1; %setting x of Start point
ys = 1; %setting y of Start point
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
elseif  (xg<0 || xg>250) || (yg<0 || yg>=250)
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
% drawnow
% plot(cNode(1),cNode(2),'*','color','red')

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
% drawnow
% plot(nNodeleft(1),nNodeleft(2),'*','color','green')
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

[Flagright, nNoderight] = mr(cNode); %Move right
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
% drawnow
% plot(nNoderight(1),nNoderight(2),'*','color','green')
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
        
[Flagup, nNodeup] = mu(cNode); %Move up
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
% drawnow
% plot(nNodeup(1),nNodeup(2),'*','color','green')
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
        
[Flagdown, nNodedown] = md(cNode); %Move down
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
% drawnow
% plot(nNodedown(1),nNodedown(2),'*','color','green')
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
        
[Flagdownleft, nNodedownleft] = mdl(cNode); %Move down and left
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
% drawnow
% plot(nNodedownleft(1),nNodedownleft(2),'*','color','green')
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
[Flagdownright, nNodedownright] = mdr(cNode); %Move down and right
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
% drawnow
% plot(nNodedownright(1),nNodedownright(2),'*','color','green')
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

[Flagupleft, nNodeupleft] = mul(cNode); %Move up and left
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
% drawnow
% plot(nNodeupleft(1),nNodeupleft(2),'*','color','green')
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

[Flagupright, nNodeupright] = mur(cNode); %Move up and right
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
% drawnow
% plot(nNodeupright(1),nNodeupright(2),'*','color','green')
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
%% Finding parent node with minimum cost        
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
%Plotting the path
j = mincost(1,2);
k = k+1;
if rem(k,10) == 0
k
end
end
q = i-1;
count = 0;
t = 1;
txt1 = '\o Start Node'; %Caption - Start node
txt2 = '\o Goal Node'; %Caption - Goal Node
%Plot the start and end point
plot(sNode(1),sNode(2),'s','color','red','markers',10)
plot(gNode(1),gNode(2),'s','color','red','markers',10)
 
text(sNode(1),sNode(2),txt1)
text(gNode(1),gNode(2),txt2)

%% Plotting the path
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

%% Transforming the points
Tr = [-7.5;-4.5]; %Transformation and rotation of x and y points from astar algorithm
Rt = [cosd(180) -sind(180); sind(180) cosd(180)]; 
%% Defining Handles
[returnCode,t_leftwheel] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking) 
[returnCode,t_rightwheel] = vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
[returnCode,terp_bot] = vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)
%% Creating a text file and defining the column headers 
fileID1 = fopen('linearx.txt','w');
fprintf(fileID1,'%6s\r\n','linear_x');
fileID2 = fopen('lineary.txt','w');
fprintf(fileID2,'%6s\r\n','linear_y');
fileID3 = fopen('linearz.txt','w');
fprintf(fileID3,'%6s\r\n','linear_z');
fileID4 = fopen('angularx.txt','w');
fprintf(fileID4,'%6s\r\n','angular_x');
fileID5 = fopen('angulary.txt','w');
fprintf(fileID5,'%6s\r\n','angular_y');
fileID6 = fopen('angularz.txt','w');
fprintf(fileID6,'%6s\r\n','angular_z');

for l= 0:(size(path,3)-1) % Comparing with the next point in the array to generate path
temppath(:,:,l+1) = [path(1,1,size(path,3)-l);path(2,1,size(path,3)-l)];
temptpath(:,:,l+1) = temppath(:,:,l+1) + Tr;
transpath(:,:,l+1) = Rt * temptpath(:,:,l+1);
end
sz = length(transpath);
a =1;
 for pp = 1: (sz)
[returnCode,angles] = vrep.simxGetObjectOrientation(clientID,terp_bot,-1,vrep.simx_opmode_blocking); %getting bot orientation
[returnCode,location] = vrep.simxGetObjectPosition(clientID,terp_bot,-1,vrep.simx_opmode_blocking); %getting bot position
     
if transpath(1,1,pp) ~= location(1)
mpoints = ((transpath(2,1,pp)-location(2))/(transpath(1,1,pp)-location(1)));
theta = atan(mpoints);
end
if transpath(1,1,pp) == location(1)
theta = (pi)/2;
end

if theta < 0
theta = theta + pi;
end     

if theta ~= (angles(2)+(pi/2))
if (theta - (angles(2)+(pi/2))) > 0
diff = (theta - (angles(2)+(pi/2)));

%% If distance between current point and next point is greater than .1 then the turtlebot will execute this part
while diff > .1       
% rotating the turtle bot    
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_leftwheel,-.5,vrep.simx_opmode_blocking); 
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_rightwheel,.5,vrep.simx_opmode_blocking);
[returnCode,angles] = vrep.simxGetObjectOrientation(clientID,terp_bot,-1,vrep.simx_opmode_blocking);
diff = (theta - (angles(2)+(pi/2)));
[returnCode,linearVelocity,angularVelocity] = vrep.simxGetObjectVelocity(clientID,terp_bot,vrep.simx_opmode_blocking)
lx(:,a) = linearVelocity(1);
ly(:,a) = linearVelocity(2);
lz(:,a) = linearVelocity(3);
angx(:,a) = angularVelocity(1);
angy(:,a) = angularVelocity(2);
angz(:,a) = angularVelocity(3);
a = a+1;
end 
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,t_leftwheel,0,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,t_rightwheel,0,vrep.simx_opmode_blocking);
end
if (theta - (angles(2)+(pi/2))) < 0
diff = (theta - (angles(2)+(pi/2)));

%% If distance between current point and next point is greater than .1 then this part will execute      
while diff < -.1 
% rotating the turtle bot  
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_leftwheel,.5,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_rightwheel,-.5,vrep.simx_opmode_blocking);
[returnCode,angles] = vrep.simxGetObjectOrientation(clientID,terp_bot,-1,vrep.simx_opmode_blocking);
diff = (theta - (angles(2)+(pi/2)));
[returnCode,linearVelocity,angularVelocity] = vrep.simxGetObjectVelocity(clientID,terp_bot,vrep.simx_opmode_blocking)
lx(:,a) = linearVelocity(1);
ly(:,a) = linearVelocity(2);
lz(:,a) = linearVelocity(3);
angx(:,a) = angularVelocity(1);
angy(:,a) = angularVelocity(2);
angz(:,a) = angularVelocity(3);
a = a+1;
end 
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_leftwheel,0,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_rightwheel,0,vrep.simx_opmode_blocking);
end       
end
[returnCode,location] = vrep.simxGetObjectPosition(clientID,terp_bot,-1,vrep.simx_opmode_blocking);
cost = sqrt((transpath(2,1,pp) - location(2))^2 + (transpath(1,1,pp) - location(1))^2);
%% If cost is greater than .8 then this part will execute     
while cost > .8
% translating the turtle bot
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_leftwheel,10,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_rightwheel,10,vrep.simx_opmode_blocking);
[returnCode,location] = vrep.simxGetObjectPosition(clientID,terp_bot,-1,vrep.simx_opmode_blocking);
cost = sqrt((transpath(2,1,pp) - location(2))^2 + (transpath(1,1,pp) - location(1))^2);
[returnCode,linearVelocity,angularVelocity] = vrep.simxGetObjectVelocity(clientID,terp_bot,vrep.simx_opmode_blocking)

lx(:,a) = linearVelocity(1);
ly(:,a) = linearVelocity(2);
lz(:,a) = linearVelocity(3);
angx(:,a) = angularVelocity(1);
angy(:,a) = angularVelocity(2);
angz(:,a) = angularVelocity(3);
a = a+1;
end
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_leftwheel,0,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxSetJointTargetVelocity(clientID,t_rightwheel,0,vrep.simx_opmode_blocking);
 end
%% Plotting individual values in the text files (Extracting velocities)
line1 = lx;
line2 = ly;
line3 = lz;
line4 = angx;
line5 = angy;
line6 = angz;
fprintf(fileID1,'%6.2f  \r\n', line1);
fprintf(fileID2,'%6.2f  \r\n', line2);
fprintf(fileID3,'%6.2f  \r\n', line3);
fprintf(fileID4,'%6.2f  \r\n', line4);
fprintf(fileID5,'%6.2f  \r\n', line5);
fprintf(fileID6,'%6.2f  \r\n', line6);

%% NOTE %% 

% I tried the combinations of all velocities in one file, combination of 2 and 3 as
%well but the values get interchanged and thus had to make all individual
%files, This was the reason for delaying in the submissions.
%plotting in combination of two velocities
% fprintf(fileID1,'%6.2f %12.2f \r\n', line1, line2);
% fprintf(fileID2,'%6.2f %12.2f \r\n', line3, line4);
% fprintf(fileID3,'%6.2f %12.2f \r\n', line5, line6);

vrep.simxFinish(-1);
end

vrep.delete();
