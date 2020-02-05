%% 路径规划总程序

function [PathStore,Path_num] = MASPP_IP(D,RobotNum,Start,Goal)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
for i = 1:RobotNum
    [PATH,path_num]=IP_solver(D,Start(i),Goal(i),i);
    PathStore{i} = PATH;
    Path_num{i} = path_num;
    MAX=max([size(PathStore{i},1),MAX]);
    MAX=MAX+1;
    H(i)=size(PathStore{i},1);
end

for i=1:RobotNum
    if size(PathStore{i},1)<MAX 
        for j = size(PathStore{i})+1:MAX
            PathStore{i}(j,1) =  PathStore{i}(H(i),1);
            PathStore{i}(j,2) =  PathStore{i}(H(i),2);
            Path_num{i}(j)=Path_num{i}(H(i));
        end
    end
    
end

temp=D;

%%
res=1;
%path_temp = [];
flag=0;

while flag == 0 % 在所有机器人达到终点前 flag置0 所有机器人达到终点后 flag置1 退出循环
%    RobotNum=3;
    if res < size(PathStore{1},1)
        for i = 1:RobotNum
            temp(PathStore{i}(res+1,1),PathStore{i}(res+1,2))=1; % 将动态地图中所有机器人下一时刻所在的节点定为障碍物
            
            Start(i)=Path_num{i}(res); % 将机器人实际所在节点作为出发点
        end
        save('temp.mat');
        res = res + 1; % 从res时刻到下一时刻res+1
        for i = 1:RobotNum
            path_temp(i) = Path_num{i}(res);
        end

        [B, I] = unique(path_temp, 'first');
        robot_coli=setdiff(1:numel(path_temp), I); % 判断有多少个机器人在时刻res+1时存在冲突
        if ~isempty(robot_coli)
            for j = 1:length(robot_coli) % 第j个机器人存在冲突
                text=' 号机器人出现冲突';
                disp([num2str(j),text]);
                if Start(j) == Goal(j)
                    text=' 号机器人已到达终点，冲突忽略';
                    disp([num2str(j),text]);  
                    continue; %% 忽略与被已经达到终点的机器人发生冲突的情况。
                end
                [PATH,Path_num_MAJ]=Modify_path(temp,Start(j),Goal(j),j);  % 第j个冲突机器人路径重新规划
                PathStore{j}([res-1,max([size(PathStore{j},1),res-1+size(PATH,1)])],:) = PATH;  %更新第j个机器人的最优路径选择集合 res-1开始更新PathStore矩阵
                Path_num{j}([res-1,max([size(Path_num{j}),res-1+size(Path_num)])],:) = Path_num_MAJ;
            end
        end

        if isequal(Start,Goal)
            flag = 1;
        end

        for i = 1:RobotNum
            temp(PathStore{i}(res,1),PathStore{i}(res,2))=0; % 释放当前节点
        end
      %  flag=1
    else
        flag=1;
    end
    
end
%%
save('Path_num.mat');
load('Path_num.mat');




