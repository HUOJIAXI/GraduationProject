%% 路径规划总程序

function [PathStore,Path_num] = MASPP_IP(D,RobotNum,Start,Goal)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;

SD=size(D,1);
Goal_ori=Goal;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
H=zeros(RobotNum,1);
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
 
        
        path_temp=zeros(RobotNum,1);
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
                    disp('交叉冲突临时释放目标节点')
                    temp(X_fin,Y_fin)=0;% 释放目标节点
                    disp(num2str(temp(X_fin,Y_fin)))
                 %   tic
                    [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(j),Goal(j),j);  % 第j个冲突机器人路径重新规划
                 %   toc
                 
                    temp_ori =temp;
                    
                    while 1
                        if RE == 0
                            break
                        end
                        
                        if RE == 1
                              disp('终点被包围，启用备用终点,重新规划路径');
                              [Goal(j),flgn,m]=GOAL_RESERVE(temp_ori,Goal(j),SD);

                              if flgn == 0
                                 disp('环境密度太大，无法找到备用终点，求解错误');
                                 return;
                              end

                              [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(j),Goal(j),j);
                              
                              if flgn==1
                                    Goal(j)=Goal(j)-m; % 返回原始终点
                              end

                              if flgn==2
                                    Goal(j)=Goal(j)+m;
                              end

                              if RE == 0
                                    disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                                    break
                              end
                        end

                         if RE == 1
                             [X_fin_it,Y_fin_it] = spread_sin(Goal(j),SD);
                             disp('该备用终点依然被包围，重新寻找备用终点')
                             temp_ori(X_fin_it,Y_fin_it)=1; % 将无法使用的备用终点排除
                         end

                    end
                    
                    PathStore{j,1}(res-1:size(PathStore{j,1},1),:)=[];
                    Path_num{j,1}(res-1:size(Path_num{j,1},2))=[];

                    PathStore{j,1}=[PathStore{j,1};PATH];
                    Path_num{j,1}=[Path_num{j,1} Path_num_MAJ];
                end
            end
        end

        %% 非交叉冲突
        z=0;
        robot_coli=cell(RobotNum,1); % 预分配空间
        
         for l=1:RobotNum
             if ~ismember(l,robot_coli{1})
                 same=path_temp(l);
                 for p=1:RobotNum
                     if same == path_temp(p) && l ~= p
                         z=z+1;
                         robot_coli{1}(z)=p;
                     end
                 end
             end
         end
            
        if ~isempty(robot_coli{1})
            for j = 1:length(robot_coli{1}) % 第j个机器人存在冲突
                text=' 号机器人出现非交叉冲突，正在寻找替代路径...';
                disp([num2str(robot_coli{1}(j)),text]);
                if Start(robot_coli{1}(j)) == Goal(robot_coli{1}(j))
                    text=' 号机器人已到达终点，冲突忽略';
                    disp([num2str(robot_coli{1}(j)),text]);  
                    continue; %% 忽略与已经达到终点的机器人发生冲突的情况。
                end
                
                % 使用备用终点，防止终点被占据
                [X_fin,Y_fin] = spread_sin(Goal(robot_coli{1}(j)),SD);
                [X_start,Y_start] = spread_sin(Start(robot_coli{1}(j)),SD);
                
                % 暂停行动，防止起点被占据
                if temp(X_start,Y_start)==1
                    disp('起点被占用，现时刻暂停');
                    PathStore{robot_coli{1}(j),1}=[PathStore{robot_coli{1}(j),1};(PathStore{robot_coli{1}(j),1}(res-1,:))];
                     break;
                end
                
                if temp(X_fin,Y_fin)==1
                     disp('终点被占用，临时释放终点');
                     temp(X_fin,Y_fin)=0;
                end
                     
                [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(robot_coli{1}(j)),Goal(robot_coli{1}(j)),robot_coli{1}(j));  % 第j个冲突机器人路径重新规划
                
                temp_ori =temp;
                while 1
                    if RE == 0
                           break
                    end
                        
                    if RE == 1
                          disp('终点被包围，启用备用终点,重新规划路径');
                          [Goal(robot_coli{1}(j)),flgn,m]=GOAL_RESERVE(temp_ori,Goal(robot_coli{1}(j)),SD);

                          if flgn == 0
                             disp('环境密度太大，无法找到备用终点，求解错误');
                             return;
                          end

                          [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(robot_coli{1}(j)),Goal(robot_coli{1}(j)),robot_coli{1}(j));
                          
                          if flgn==1
                            Goal(robot_coli{1}(j))=Goal(robot_coli{1}(j))-m; % 返回原始终点
                          end

                          if flgn==2
                             Goal(robot_coli{1}(j))=Goal(robot_coli{1}(j))+m;
                          end
                    end
                   
                    
                     if RE == 0
                            disp('备用终点启用成功，已生成备用路径，已切换为原始终点');     
                            break
                     end

                     if RE == 1
                         [X_fin_it,Y_fin_it] = spread_sin(Goal(robot_coli{1}(j)),SD);
                         disp('该备用终点依然被包围，重新寻找备用终点')
                         temp_ori(X_fin_it,Y_fin_it)=1; % 将无法使用的备用终点排除
                     end
                 
                end
                            

                PathStore{robot_coli{1}(j),1}(res-1:size(PathStore{robot_coli{1}(j),1},1),:)=[];
                Path_num{robot_coli{1}(j),1}(res-1:size(Path_num{robot_coli{1}(j),1},2))=[];
                
                PathStore{robot_coli{1}(j),1}=[PathStore{robot_coli{1}(j),1};PATH];
                Path_num{robot_coli{1}(j),1}=[Path_num{robot_coli{1}(j),1} Path_num_MAJ]; 
                
%                 if Fn == 1
%                     temp(X_fin,Y_fin)=1; % 恢复被清空的终点
%                 end
                
            end
        end
        
        %% 节点释放

        for i = 1:RobotNum
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
        text = ' 号机器人所有无碰撞路径已规划完成';
        disp([num2str(i),text]);
    end
    
end

%%

save('Path_num.mat');
save('PathStore.mat');
load('Path_num.mat');




