%% 主函数
% Version 1.0
% Author : HUO JIAXI
%% 初始化环境
clear;
clc;
D = load('tsp_map.txt'); 
temp = D;
l = 2;
t = 143;
%% 求解
tic
[PATH,temp]=IP_solver(temp,l,t);
toc


