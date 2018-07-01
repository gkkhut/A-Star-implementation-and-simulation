function [in] = constraints(m,n)
%% Square 
xs = [55,55,105,105];
ys = [67.5,112.5,112.5,67.5];

if m>=55 && m<=105 && n>67.5 && n<=112.5
ins = true; %In square status check
else
ins = false; %In square status check
end

%% polygon
x1 = 120; y1 = 55;
x2 = 145; y2 = 14;
x3 = 158; y3 = 51;
x4 = 165; y4 = 89;
x5 = 188; y5 = 51;
x6 = 168; y6 = 14;

x = m;
eqn1 = y1 + ((y2-y1)*(x-x1))/(x2-x1);
eqn2 = y1 + ((y3-y1)*(x-x1))/(x3-x1);
eqn3 = y3 + ((y2-y3)*(x-x3))/(x2-x3);
eqn4 = y3 + ((y4-y3)*(x-x3))/(x4-x3);
eqn5 = y4 + ((y5-y4)*(x-x4))/(x5-x4);
eqn6 = y5 + ((y6-y5)*(x-x5))/(x6-x5);
eqn7 = y6 + ((y2-y6)*(x-x6))/(x2-x6);
eqn8 = y3 + ((y5-y3)*(x-x3))/(x5-x3);
eqn9 = y3 + ((y6-y3)*(x-x3))/(x6-x3);

if (n>=eqn1 && n<=eqn2 && n>=eqn3) | (n<=eqn3 && n>=eqn7 && n<=eqn9) |  (n<=eqn4 && n>=eqn8 && n<eqn5) || (n>=eqn9 && n>=eqn6 && n<=eqn8)
inp = true; %In polygon status check
else
inp = false; %In polygon status check
end
% circle = [165 105 30 30];
%% circle 
r = 180; z = 120;
y = ((m-r)^2 + (n-z)^2)^(0.5);

if y <= 15
inc = true; %In circle status check
else
inc = false; %In circle status check
end
in = ins | inp | inc; % In square, polygon and circle
