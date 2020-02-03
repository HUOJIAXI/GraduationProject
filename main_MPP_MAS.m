%% 主函数
% Version 1.0
% Author : HUO JIAXI
%% 初始化环境
clear;
clc;
D = load('tsp_map.txt'); 
temp = D;
Start = [2,8];
Goal = [143,96];
RobotNum = 2;
%% 求解
tic
PathStore=MASPP_IP(D,RobotNum,Start,Goal);
toc
save('PathStore.mat');


