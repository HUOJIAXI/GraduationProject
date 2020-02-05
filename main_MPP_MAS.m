%% 主函数
% Version 3.0
% Author : HUO JIAXI
%% 初始化环境
clear;
clc;
D = load('tsp_map.txt'); 
m = size(D,1);
% 判断是否存在起始点在障碍物处的情况
Start = [16,3,9,8,49,1];
Goal = [121,123,143,96,131,24]; % 1号机器人终点121终点会冲突 111没有问题
flag=0;
COLI_START=zeros(length(Start),1);
COLI_FIN=zeros(length(Goal),1);

for i = 1:length(Start)
    [X,Y]=spread(Start,m);
    [X_F,Y_F]=spread(Goal,m);
    %for j = 1:length(X)
    if D(X(i),Y(i))==1
        flag = 1;
        COLI_START=[COLI_START;i];
    end
    if D(X_F(i),Y_F(i))==1
        flag = 1;
        COLI_FIN=[COLI_FIN;i];
    end
    %end
end

if flag ==1
    return;
end

%Start = 2;
%Goal = 143;
RobotNum = length(Start);
%% 求解
 tic
 [PathStore,Path_num]=MASPP_IP(D,RobotNum,Start,Goal);
 toc
 save('PathStore.mat');
%% 仿真视频存储
plotdynamic(D,PathStore,RobotNum,Start,Goal);

%% 原始环境
mapdesigner(fliplr(D),2);

for i = 1:RobotNum
    plot((PathStore{i}(:,2)-1/2),(PathStore{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10) % 将所有机器人的路径显示在图中。
    if(i==RobotNum)
        break;
    else
        hold on;
    end
end

%% 打印各个机器人的行走顺序
for i = 1:RobotNum
    x_fig=zeros((length(Path_num{i})-1),1);
    y_fig=zeros((length(Path_num{i})-1),1);
    for j = 1:(length(Path_num{i})-1)
        x_fig(j)=Path_num{i}(j);
        y_fig(j)=Path_num{i}(j+1);
    end
    
    figure(2+i)
    str=['robot=',num2str(i)];
    %G=digraph(A,'OmitSelfLoops');
    G=digraph(x_fig,y_fig,'OmitSelfLoops');
    plot(G,'Layout','layered');
    title(str);
end


