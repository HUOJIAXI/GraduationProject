%% 路径规划总程序

function [PathStore,Path_num] = MASPP_IP(D,RobotNum,Start,Goal)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;

SD=size(D,1);
Goal_ori=Goal;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
for i = 1:RobotNum
    [PATH,path_num]=IP_solver(D,Start(i),Goal(i),i);
    PathStore{i,1} = PATH;
    Path_num{i,1} = path_num;
    MAX=max([size(PathStore{i,1},1),MAX]);
%    MAX=MAX+1;
    H(i)=size(PathStore{i,1},1);
end

MAX = MAX+1;

for i=1:RobotNum
    if size(PathStore{i,1},1)<MAX 
        SI=size(PathStore{i,1});
        for j = SI+1:MAX
            PathStore{i,1}(j,1) =  PathStore{i,1}(H(i),1);
            PathStore{i,1}(j,2) =  PathStore{i,1}(H(i),2);
            Path_num{i,1}(j)    =  Path_num{i,1}(H(i));
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
    MAX = 0;
    for i = 1:RobotNum
    %     [PATH,path_num]=IP_solver(D,Start(i),Goal(i),i);
    %     PathStore{i} = PATH;
    %     Path_num{i} = path_num;
        MAX=max([size(PathStore{i,1},1),MAX]);
    %    MAX=MAX+1;
        H(i)=size(PathStore{i,1},1);
    end
    
    MAX =MAX+1;

    for i=1:RobotNum
        if size(PathStore{i,1},1)<MAX 
            SI=size(PathStore{i,1});
            for j = (SI+1):MAX
                PathStore{i,1}(j,1) =  PathStore{i,1}(H(i),1);
                PathStore{i,1}(j,2) =  PathStore{i,1}(H(i),2);
                Path_num{i,1}(j)      =  Path_num{i,1}(H(i));
            end
        end

    end
    
    if res < size(PathStore{3,1},1)    
        %% 解决两种冲突，迎面冲突和转角冲突
        for i = 1:RobotNum
            temp(PathStore{i,1}(res+1,1),PathStore{i,1}(res+1,2)) = 1; % 将动态地图中所有机器人下一时刻所在的节点定为障碍物
            
            Start(i)=Path_num{i,1}(res); % 将机器人实际所在节点作为出发点
        end
       
        
        res = res + 1; % 从res时刻到下一时刻res+1
        %save('temp.mat');
 
        
      
        for i = 1:RobotNum
            path_temp(i) = Path_num{i,1}(res); % 储存下一节点
        end
        
        %% 交叉冲突
        for i =1:RobotNum
            for j = 1:RobotNum
                if (abs(path_temp(i)-path_temp(j))==SD && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==SD)||(abs(path_temp(i)-path_temp(j))==1 && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==1)
                    
                    text=' 号机器人出现交叉冲突，正在寻找替代路径...';
                    disp([num2str(j),text]);
                    [X_start,Y_start] = spread_sin(Start(j),SD);
                    disp(num2str(temp(X_start,Y_start)))
                    disp('交叉冲突释放起始节点')
                    temp(X_start,Y_start)=0; % 释放起始节点
                    disp(num2str(temp(X_start,Y_start)))

                    [X_fin,Y_fin] = spread_sin(Goal(j),SD);
                    disp(num2str(temp(X_fin,Y_fin)))
                    disp('交叉冲突释放目标节点')
                    temp(X_fin,Y_fin)=0;% 释放目标节点
                    disp(num2str(temp(X_fin,Y_fin)))
                 %   tic
                    [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(j),Goal(j),j);  % 第j个冲突机器人路径重新规划
                 %   toc

                    %
                    if RE == 1
                        % 对于某些情况，终点被包围，可能需要启用备用终点，现版本仍然无法避免，只能对于四周被包围的情况，如果周围存在0点的话，无法使用
                        disp('终点被包围，需要启用备用终点');
                        flgn=0;
                        disp('备用终点启用');
                        for m = 1:SD*SD-Goal(j)
                        [X_fin_pos,Y_fin_pos]=spread_sin(Goal(j)+m,SD);
                            if temp(X_fin_pos,Y_fin_pos)==0
                                Goal(j)=Goal(j)+m;
                                flgn=1;
                                break;
                            end
                        end

                        if flgn==0
                            for m = 1:Goal(j)-1
                                [X_fin_neg,Y_fin_neg]=spread_sin(Goal(j)-m,SD);
                                if temp(X_fin_neg,Y_fin_neg)==0
                                    Goal(j)=Goal(j)-m;
                                    flgn=2;
                                    break;
                                end
                            end
                        end

                        if flgn==0
                            return
                        end               
                        
                        [~,PATH,Path_num_MAJ]=Modify_path(temp,Start(j),Goal(j),j);
                        
                        if flgn==1
                            Goal(j)=Goal(j)-m;
                        end
                        
                        if flgn==2
                            Goal(j)=Goal(j)+m;
                        end
                        
                    end
                    %
                    
                    PathStore{j,1}(res-1:size(PathStore{j,1},1),:)=[];
                    Path_num{j,1}(res-1:size(Path_num{j,1},2))=[];

                    PathStore{j,1}=[PathStore{j,1};PATH];
                    Path_num{j,1}=[Path_num{j,1} Path_num_MAJ];
                end
            end
        end

%         I = unique(path_temp, 'first'); % path_temp中的去重复后存于I
%         robot_coli=setdiff(1:numel(path_temp), I); % 判断有多少个机器人在时刻res+1时存在冲突（转角冲突），缺少迎面冲突的问题，需要再加入一个限制
       

        %% 非交叉冲突
        z=0;
        robot_coli=[];
        
         for l=1:RobotNum
             if ~ismember(l,robot_coli)
                 same=path_temp(l);
                 for p=1:RobotNum
                     if same == path_temp(p) && l ~= p
                         z=z+1;
                         robot_coli(z)=p;
                     end
                 end
             end
         end
            
        if ~isempty(robot_coli)
            for j = 1:length(robot_coli) % 第j个机器人存在冲突
                text=' 号机器人出现非交叉冲突，正在寻找替代路径...';
                disp([num2str(robot_coli(j)),text]);
                if Start(robot_coli(j)) == Goal(robot_coli(j))
                    text=' 号机器人已到达终点，冲突忽略';
                    disp([num2str(robot_coli(j)),text]);  
                    continue; %% 忽略与已经达到终点的机器人发生冲突的情况。
                end
                
                % 使用备用终点，防止终点被占据
                [X_fin,Y_fin] = spread_sin(Goal(robot_coli(j)),SD);
                [X_start,Y_start] = spread_sin(Start(robot_coli(j)),SD);
                
                % 暂停行动，防止起点被占据
                if temp(X_start,Y_start)==1
                    disp('起点被占用，现时刻暂停');
                    PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1};(PathStore{robot_coli(j),1}(res-1,:))];
                     break;
                end
                
                flgn=0;
                if temp(X_fin,Y_fin)==1
                    disp('备用终点启用');
                    for m = 1:SD*SD-Goal(robot_coli(j))
                    [X_fin_pos,Y_fin_pos]=spread_sin(Goal(robot_coli(j))+m,SD);
                        if temp(X_fin_pos,Y_fin_pos)==0
                            Goal(robot_coli(j))=Goal(robot_coli(j))+m;
                            flgn=1;
                            break;
                        end
                    end
                    
                    if flgn==0
                        for m = 1:Goal(robot_coli(j))-1
                            [X_fin_neg,Y_fin_neg]=spread_sin(Goal(robot_coli(j))-m,SD);
                            if temp(X_fin_neg,Y_fin_neg)==0
                                Goal(robot_coli(j))=Goal(robot_coli(j))-m;
                                flgn=1;
                                break;
                            end
                        end
                    end
                    
                    if flgn==0
                        return
                    end
                end                               
                %
                 
                [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(robot_coli(j)),Goal(robot_coli(j)),robot_coli(j));  % 第j个冲突机器人路径重新规划
                
                if RE == 1
                    disp('终点被包围，启用备用终点');
                end
                
%                 PathStore{j}([res-1,max([size(PathStore{j,1},1),res-1+size(PATH,1)])],:) = PATH;  %更新第j个机器人的最优路径选择集合 res-1开始更新PathStore矩阵
%                 Path_num{j}([res-1,max([size(Path_num{j,1}),res-1+size(Path_num_MAJ)])],:) = Path_num_MAJ;

                PathStore{robot_coli(j),1}(res-1:size(PathStore{robot_coli(j),1},1),:)=[];
                Path_num{robot_coli(j),1}(res-1:size(Path_num{robot_coli(j),1},2))=[];
                
                PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1};PATH];
                Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ]; %% 2和3在12时碰撞 Path_num有问题
                
%                 if Fn == 1
%                     temp(X_fin,Y_fin)=1; % 恢复被清空的终点
%                 end
                
            end
        end
        
        %% 节点释放

        for i = 1:RobotNum
            disp('释放当前节点')
            temp(PathStore{i}(res,1),PathStore{i}(res,2))=0; % 释放当前节点
        end
        
        if isequal(Start,Goal)
        %if res==size(PathStore{3,1},1)
             disp('路径修改全部完成')
             flag = 1;
        end
        
      %  flag=1
    else
        flag=1;
    end
    
end

%% 补充规划变更过的终点
inte=Goal-Goal_ori;
for i = 1:RobotNum
    if inte(i) ~= 0
        disp('开始规划剩余路径');
        [~,PATH_sup,Path_num_sup]=Modify_path(temp,Goal(i),Goal_ori(i),i);
        PathStore{i,1}=[PathStore{i,1};PATH_sup];
        Path_num{i,1}=[Path_num{i,1} Path_num_sup];
    else
        disp('所有路径已规划完成')
    end
    
end

%%

save('Path_num.mat');
save('PathStore.mat');
load('Path_num.mat');




