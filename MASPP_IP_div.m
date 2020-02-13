%% 本版本基于MASPP_IP，探究通过子图分割降低路径修改时延

function [PathStore,Path_num] = MASPP_IP_div(D,RobotNum,Start,Goal)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;

SD=size(D,1);
%Goal_ori=Goal;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
for i = 1:RobotNum
    [PATH,path_num] = ori_path(D,Start(i),Goal(i),SD,i);
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
    
    if res < MAX   
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
%                     disp(num2str(temp(X_fin,Y_fin)))
%                     disp('交叉冲突临时释放目标节点')
%                     temp(X_fin,Y_fin)=0;% 释放目标节点
%                     disp(num2str(temp(X_fin,Y_fin)))

                     encarde = 2;
                     
                     if X_start > encarde && Y_start > encarde
                             disp('横纵坐标满足大于encarde的要求')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                 disp('终点在框外')
                                [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp]=op_modify_path(temp,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD);
                             else
                                 disp('终点在框内')
                                 [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
                             end
                            
                     else
                         % 补充原始节点的横纵坐标小于等于3的情况
                             disp('横纵坐标不满足大于encarde的要求')
                            [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
                     
                     end  
                end
            end
        end

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
                
                if temp(X_fin,Y_fin)==1
                     disp('终点被占用，临时释放终点');
                     temp(X_fin,Y_fin)=0;
                end
                
                 if X_start > encarde && Y_start > encarde
                             disp('横纵坐标满足大于encarde的要求')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                disp('终点在框内')
                                [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp]=op_modify_path(temp,X_start,Y_start,X_fin,Y_fin,Start(robot_coli(j)),Goal(robot_coli(j)),Path_num{robot_coli(j),1},PathStore{robot_coli(j),1},robot_coli(j),res,SD);
                             else
                                disp('终点在框外')
                                [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp] = op_modify_sup(temp,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),res,robot_coli(j),SD,D);
                             end
                            
                 else
                         % 补充原始节点的横纵坐标小于等于3的情况
                            [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp] = op_modify_sup(temp,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),res,robot_coli(j),SD,D);
                     
                 end       
                
            end
        end
        
        %% 节点释放

        for i = 1:RobotNum
            temp(PathStore{i}(res,1),PathStore{i}(res,2))=0; % 释放当前节点
        end
        
        if isequal(Start,Goal)
             disp('路径修改全部完成')
             flag = 1;
        end
        
    else
        flag=1;
    end
    
end

%% 补充规划变更过的终点
inte=Goal-Start;
for i = 1:RobotNum
    if inte(i) ~= 0
        disp('开始规划剩余路径');
        
        [X_start,Y_start] = spread_sin(Start(i),SD);
        [X_fin,Y_fin] = spread_sin(Goal(i),SD);
        squ=max(abs(X_start-X_fin),abs(Y_start-Y_fin));
        ini_x=min(X_start,X_fin);
        ini_y=min(Y_start,Y_fin);
        
        Start_op_x=X_start-ini_x+1;
        Start_op_y=Y_start-ini_y+1;
        Goal_op_x =X_fin  -ini_x+1;
        Goal_op_y =Y_fin  -ini_y+1;
        Start_op=Start_op_y+(Start_op_x-1)*(squ+1);
        Goal_op = Goal_op_y+(Goal_op_x-1)*(squ+1);
        
        temp_D=D(ini_x:ini_x+squ,ini_y:ini_y+squ);
        [PATH_sup,~]=IP_solver(temp_D,Start_op,Goal_op,i);
        PATH_sup(:,1)=PATH_sup(:,1)+ini_x-1;
        PATH_sup(:,2)=PATH_sup(:,2)+ini_y-1;
        %path_num=path_num+(ini_y-1)+(ini_x-1)*SD;
        Path_num_sup=(PATH_sup(:,2)+(PATH_sup(:,1)-1)*SD)';
       
        PathStore{i,1}=[PathStore{i,1};PATH_sup];
        Path_num{i,1}=[Path_num{i,1} Path_num_sup];
        text = ' 号机器人所有无碰撞路径已规划完成';
        disp([num2str(i),text]);
    else
        text = ' 号机器人所有无碰撞路径已规划完成';
        disp([num2str(i),text]);
    end
    
end

%%

save('Path_num.mat');
save('PathStore.mat');
