MATLAB program is included in the code folder. 
The program generates the an optimum path from start point to end points
avoiding obstacles using A* Algorithm

Steps to run the program:
Extract the files to a folder
1) Open MATLAB and open [astar.m] file which is in the Code folder.
2) Make sure all the files are in the code directory.
3) Change the working directory to Code folder.
4) Open vrep and make changes in child script and start the simulation window, 
(You need to make changes in "non-threaded child script(turtlebot)" - by everytime adding the simExtRemoteApiStart(19999) line.
5) Then run the main program.
6) Select the goal point and check the simulation in vrep
7) The optimum path will be generated using A* algorithm once the execution is completed(reached the goal point).
8) Details regarding the nodes and nodeinfo are available at the variable workspace - nodes and nodesinfo.
9) Output velocities are in the output folder and the error combinations are also included in (errorfolder)Output folder
10) I tried the combinations of all velocities in one file, combination of 2 and 3 as well but the values get interchanged
   and thus had to make all individual files, This was the reason for delaying in the submissions.


Contact: 
For more details contact Gunjan Khut(gkkhut@terpmail.umd.edu).
