clear;
clc;
D = load('dist_map.txt'); 
m = size(D,1);
% 判断是否存在起始点在障碍物处的情况
% Start = [1,5,15,10,18,20,3,29,45];
% Goal = [24,21,22,26,49,42,38,43,4];
RobotNum=9;
[Start,Goal]=rand_Goal_Start(D,RobotNum);
%RobotNum = size(Start,2);
size_D=size(D,1);

tic
 [PathStore,Path_num]=IP_solver_single_way(D,Start,Goal,RobotNum,size_D);
toc

plotdynamic(D,PathStore,Path_num,RobotNum,Start,Goal);

mapdesigner(fliplr(D),2);

show=ceil(sqrt(RobotNum));

for i = 1:RobotNum
        mapdesigner_show(fliplr(D),i,show,0); % 最后一个参数控制行数 是否需要-1
        plot((PathStore{i}(:,2)-1/2),(PathStore{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10) ;% 将所有机器人的路径显示在图中。
        str=['robot=',num2str(i)];
        title(str);
end

save('PathStore.mat')
save('Path_num.mat')