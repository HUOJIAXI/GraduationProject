clear;
clc;
D = load('dist_map.txt'); 
m = size(D,1);
% 判断是否存在起始点在障碍物处的情况
Start = 1;
Goal = 9;
numrobot = 1;
size_D=size(D,1);

tic
 [PATH,Path]=IP_solver_single_way(D,Start,Goal,numrobot,size_D);
toc