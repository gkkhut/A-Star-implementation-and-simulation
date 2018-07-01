function [in] = constraints(m,n)
%% Square 1
xs1= [1.575,1.575,2.375,2.375];
ys1=[7.375,9.375,9.375,7.375];

if m>=1.555 && m<=2.395 && n>7.355 && n<=9.395
ins1 = true; %In square status check
else
ins1 = false; %In square status check
end

%% Square 2
xs2 = [2.775,2.775,3.575,3.575];
ys2 =[7.375,9.375,9.375,7.375];

if m>=2.755 && m<=3.595 && n>7.355 && n<=9.395
ins2 = true; %In square status check
else
ins2 = false; %In square status check
end

%% Square 3
xs3= [12.05,12.05,13.65,13.65];
ys3=[8.55,9.65,9.65,8.55];

if m>=11.85 && m<=13.85 && n>8.53 && n<=9.67
ins3 = true; %In square status check
else
ins3 = false; %In square status check
end

%% Square 4
xs4= [14.05,14.05,14.85,14.85];
ys4=[4.275,6.275,6.275,4.275];

if m>=13.85 && m<=15.05 && n>4.255 && n<=6.295
ins4 = true; %In square status check
else
ins4 = false; %In square status check
end

%% Square 5
xs5= [14.05,14.05,14.85,14.85];
ys5=[2.275,4.275,4.275,2.275];

if m>=13.85 && m<=15.05 && n>2.255 && n<=4.295
ins5 = true; %In square status check
else
ins5 = false; %In square status check
end

%% Square 6
xs6= [5.5525,5.5525,7.1525,7.1525];
ys6=[4.2,5.8,5.8,4.2];

if m>=5.5325 && m<=7.1725 && n>4.18 && n<=5.82
ins6 = true; %In square status check
else
ins6 = false; %In square status check
end

%% Square 7
xs7= [9.3,9.3,10.9,10.9];
ys7=[4.2,5.8,5.8,4.2];

if m>=9.28 && m<=11.10 && n>4.18 && n<=5.82
ins7 = true; %In square status check
else
ins7 = false; %In square status check
end

%% circles
%circle 1 
r = 6.3525; z = 5.8;
y = ((m-r)^2 + (n-z)^2)^(0.5);
if y <= 1
inc1 = true; %In circle status check
else
inc1 = false; %In circle status check
end
%circle 2
r = 6.3525; z = 4.2;
y = ((m-r)^2 + (n-z)^2)^(0.5);
if y <= 1
inc2 = true; %In circle status check
else
inc2 = false; %In circle status check
end
%circle 3
r = 10.1; z = 5.8;
y = ((m-r)^2 + (n-z)^2)^(0.5);
if y <= 1
inc3 = true; %In circle status check
else
inc3 = false; %In circle status check
end
%circle 4
r = 10.1; z = 4.2;
y = ((m-r)^2 + (n-z)^2)^(0.5);
if y <= 1
inc4 = true; %In circle status check
else
inc4 = false; %In circle status check
end

in =  ins1 | ins2 |ins3 |ins4 |ins5 |ins6 |ins7 | inc1 | inc2 | inc3 | inc4; % In squares and circles