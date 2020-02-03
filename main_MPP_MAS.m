%% 主函数
% Version 1.0
% Author : HUO JIAXI
%% 初始化环境
clear;
clc;
D = load('tsp_map.txt'); 
temp = D;
Start = 2;
Goal = 143;
RobotNum = 1;
%% 求解
tic
PathStore=MASPP_IP(D,RobotNum,Start,Goal);
toc
save('PathStore.mat');


