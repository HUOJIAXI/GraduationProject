clear;
clc;
D = load('tsp_dist.txt'); 
m = size(D,1);
% 判断是否存在起始点在障碍物处的情况
% Start = [1,5,15,10,18,20,3,29,45];
% Goal = [24,21,22,26,49,42,38,43,4];
RobotNum=16;
[Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum);
%RobotNum = size(Start,2);

% [Start,Goal,start_sp,goal_sp,D_reduit] = reduit(Start_ori,Goal_ori,D);
% size_D=size(D_reduit,1);
size_D=size(D,1);


tic
 [PathStore,Path_num]=IP_solver_single_way_V2(D,Start_ori,Goal_ori,RobotNum,size_D);
toc

% [PathStore_new,Path_num_new]=broaden(PathStore,D,RobotNum,Start_ori,Goal_ori);
% 
% exam(Path_num_new,Start_ori,Goal_ori,RobotNum);
% 
plotdynamic(D,PathStore,Path_num,RobotNum,Start_ori,Goal_ori);
% 
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