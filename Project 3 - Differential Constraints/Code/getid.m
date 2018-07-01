function [id]= getid(newnode)
% This function is used to generate a unique ID for each node. 
a = floor(newnode(1));
b = floor(newnode(2));
id = strcat(num2str(a),num2str(b));
id = str2num(id);
end