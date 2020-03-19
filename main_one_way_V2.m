clear;
clc;
D = load('tsp_dist_broad.txt'); 
m = size(D,1);
RobotNum=4;
[Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum);
%RobotNum = size(Start,2);

[Start,Goal,start_sp,goal_sp,D_reduit] = reduit(Start_ori,Goal_ori,D);
% disp(D_reduit)
% disp(Start)
% disp(Goal)
size_D=size(D_reduit,2);
% size_D=size(D,2);

diary('res_16.txt');

test = cat(2,Start',Goal');
equal=[];
Start_new=[];
Goal_new=[];
equal_now=[];
for i = 1:RobotNum-1
    test_o=test(i,:);
    if ~ismember(i,equal)
        for j = i+1:RobotNum

                test_o_i=test(j,:);
                if (test_o-test_o_i)==0

                    equal=[equal j];

                end
        end
    end
end

for j =1:RobotNum

    if ~ismember(j,equal)
        Start_new = [Start_new Start(j)];
        Goal_new  = [Goal_new Goal(j)];
    end

end

RobotNum_new=length(Start_new);

if length(Start_new)-length(Start)==0
    disp('不存在重合起点终点');
else
    disp('存在重合起点终点');
end

tic
 [PathStore,Path_num]=IP_solver_single_way_V2(D_reduit,Start_new,Goal_new,RobotNum_new,size_D);
%  [PathStore,Path_num]=IP_solver_single_way_V2(D,Start_ori,Goal_ori,RobotNum,size_D);
toc
diary('off');

PathStore_re=cell(RobotNum,1);
Path_num_re=cell(RobotNum,1);

for i = 1:RobotNum
    start_find = find(Start_new==Start(i));
    goal_find = find(Goal_new==Goal(i));
    
    for j = 1:length(start_find)
        for m = 1:length(goal_find)
            if start_find(j)==goal_find(m)
                findindex=start_find(j);
            end
        end
    end
 
       PathStore_re{i,1}=PathStore{findindex,1};
       Path_num_re{i,1}=Path_num{findindex,1};
    
end
PathStore=PathStore_re;
Path_num=Path_num_re;

[PathStore_new,Path_num_new]=broaden(PathStore,D,RobotNum,Start_ori,Goal_ori);

exam(Path_num_new,Start_ori,Goal_ori,RobotNum);

% plotdynamic(D,PathStore,Path_num,RobotNum,Start_ori,Goal_ori);
% 

plotdynamic(D,PathStore_new,Path_num_new,RobotNum,Start_ori,Goal_ori);

mapdesigner(fliplr(D),2);

show=ceil(sqrt(RobotNum));

for i = 1:RobotNum
        mapdesigner_show(fliplr(D),i,show,0); % 最后一个参数控制行数 是否需要-1
        plot((PathStore_new{i}(:,2)-1/2),(PathStore_new{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10) ;% 将所有机器人的路径显示在图中。
        str=['robot=',num2str(i)];
        title(str);
end

% mapdesigner(fliplr(D),2);
% 
% show=ceil(sqrt(RobotNum));
% 
% for i = 1:RobotNum
%         mapdesigner_show(fliplr(D),i,show,0); % 最后一个参数控制行数 是否需要-1
%         plot((PathStore{i}(:,2)-1/2),(PathStore{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10) ;% 将所有机器人的路径显示在图中。
%         str=['robot=',num2str(i)];
%         title(str);
% end

save('PathStore.mat')
save('Path_num.mat')