%% 主函数
% Version 7.1
% Author : HUO JIAXI
% 现行版本能够解决终点被包围的情况，7.1版本改进了7.0版本中出现的跳跃的情况，并且解决了可能会发生的在某点卡死的情况，在13个机器人的环境下，运行时间进一步减少，运行时间为195.06s。但是7.1版本还存在着出现子循环的情况，造成了一定的资源浪费，但是最后能够到达>    最终目的地，还需要在之后的版本中进行改进。
%% 初始化环境
clear;
clc;
D = load('tsp_map.txt'); 
m = size(D,1);
% 判断是否存在起始点在障碍物处的情况
Start = [16,135,18,7,42,33,8,111,144,129,150,1,12,121]; % 113
Goal = [121,74,135,96,131,45,35,141,150,100,168,94,133,137]; % 135
encarde=2; %启发式算法参数2 3
control=15; % 路径整理调整参数
% Start = [16,133,9,8,49,1,7];
% Goal = [121,74,143,96,131,24,35]; 

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
    text='终点或节点不可以选在障碍物处';
    disp(text);
    return;
end

%Start = 2;
%Goal = 143;
RobotNum = length(Start);
%% 求解
 tic
 [PathStore,Path_num]=MASPP_IP_div(D,RobotNum,Start,Goal,encarde,control);
 toc
 
%% 仿真视频存储
plotdynamic(D,PathStore,Path_num,RobotNum,Start,Goal);

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

D_after=D;
for i = 1:RobotNum
    D_after(PathStore{i}(:,1),PathStore{i}(:,2))=1;
end

D_mes=D_after-D;

path_go=find(D_mes==1);
dis_total=length(path_go);
disp('系统总路程（不包括重复经过的点）')
disp(dis_total)

