%% 本版本基于MASPP_IP，探究通过子图分割降低路径修改时延

function [PathStore,Path_num] = MASPP_IP_div_op(D,RobotNum,Start,Goal,encarde,control)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;

SD=size(D,2);
%Goal_ori=Goal;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
for i = 1:RobotNum
    [PathStore,Path_num] = ori_path_am(Start,Goal,RobotNum,D);
    text=' 号机器人原始路径规划完成';
    disp([num2str(i),text]);
    
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

%%
res=1;
%path_temp = [];
flag=0;

while flag == 0 % 在所有机器人达到终点前 flag置0 所有机器人达到终点后 flag置1 退出循环
    if res < 30
        %% 解决两种冲突，迎面冲突和转角冲突
        for i = 1:RobotNum
            Start(i)=Path_num{i,1}(res); % 将机器人实际所在节点作为出发点
        end
        res = res + 1; % 从res时刻到下一时刻res+1
  
        path_temp=zeros(RobotNum,1);
        path_num_temp=zeros(RobotNum,2);
        for i = 1:RobotNum
            path_temp(i) = Path_num{i,1}(res); % 储存下一节点
            path_num_temp(i,1)=PathStore{i,1}(res,1);
            path_num_temp(i,2)=PathStore{i,1}(res,2);
        end
        
        %% 交叉冲突
        for i =1:RobotNum
            for j = 1:RobotNum
                if (Path_num{i,1}(res-1)+Path_num{i,1}(res))==(Path_num{j,1}(res-1)+Path_num{j,1}(res))&&i~=j
%                 if (abs(path_temp(i)-path_temp(j))==SD && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==SD)||(abs(path_temp(i)-path_temp(j))==1 && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==1)
                    
%                     col=Path_num{i,1}(res-1)+Path_num{i,1}(res)/2;
%                     [col_x,col_y]=spread_sin(col,size(D,2));

                    temp_dir=D;
%                     temp_dir(col_x,col_y) = 1;
%                     
                    for rob = 1:RobotNum
                        temp_dir(PathStore{rob,1}(res-1,1),PathStore{rob,1}(res-1,2)) = 1; % 将动态地图中所有机器人此时刻所在的节点定为障碍物
                    end
                    if Start(j) == Goal(j)||Start(i) == Goal(i)
                    text=' 号机器人已到达终点，冲突忽略';
                    disp([num2str(j),text]);  
                    continue; %% 忽略与已经达到终点的机器人发生冲突的情况。
                    end
                    text=' 号机器人出现交叉冲突，正在寻找替代路径...';
                    disp([num2str(j),text]);
                    [X_start,Y_start] = spread_sin(Start(j),SD);

                     [X_fin,Y_fin] = spread_sin(Goal(j),SD);
                     
                     if X_start > encarde && Y_start > encarde && X_start+encarde <= SD && Y_start+encarde <= SD
                             disp('横纵坐标满足大于边框的要求')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                 disp('终点在框外')
                                [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),~]=op_modify_path(D,temp_dir,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD,encarde);
                             elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                 disp('终点在框内') %% 可以尝试启发式方法，将求解范围缩小，修改temp，借鉴原始路径的启发式方法
                                 [PathStore{j,1},Path_num{j,1},~] = ori_path_op(D,temp_dir,X_start,Y_start,X_fin,Y_fin,PathStore{j,1},Path_num{j,1},SD,j,encarde,res);
                             else
                                 disp('终点在框外') 
                                 [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),~] = op_modify_path(D,temp_dir,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD,encarde);
                             end
                            
                     else
                         % 补充原始节点的横纵坐标小于等于3的情况，需要增加额外的启发式算法
                             disp('横或纵坐标不满足大于边框的要求')                           
                              % 优化边界冲突处理，将原环境拓展
                             one_add_y=ones(SD,encarde);
                             one_add_x=ones(encarde,SD+2*encarde);
                             temp_am=temp_dir;
%                              disp(temp)
                             D_am=D;
                             
                             temp_am_op_1=[one_add_y temp_am one_add_y];
                             temp_am_op=[one_add_x;temp_am_op_1;one_add_x];
                             
                             D_am_op_1=[one_add_y D_am one_add_y];
                             D_am_op=[one_add_x;D_am_op_1;one_add_x];
                             
                       %      [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
                             
                              disp('环境图进行扩展处理')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                 disp('终点在框外')
%                                  disp(temp_am_op)
                                [PathStore{j,1},Path_num{j,1},Start(j),Goal(j)]=op_modify_path_am(temp_dir,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD+2*encarde,SD,encarde);
                             elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                 disp('终点在框内') %% 可以尝试启发式方法，将求解范围缩小，修改temp，借鉴原始路径的启发式方法
                                 [PathStore{j,1},Path_num{j,1}] = ori_path_op_am(D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,PathStore{j,1},Path_num{j,1},SD,j,encarde,res);
%                                if RE == 1
%                                   disp('启发式算法失效')
%                                  [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
%                                end
                             else
                                 disp('终点在框外') 
                                  [PathStore{j,1},Path_num{j,1},Start(j),Goal(j)]=op_modify_path_am(temp_dir,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD+2*encarde,SD,encarde);
                             end
                             
%                      temp(X_start,Y_start)=1; %     恢复起点
                     end  
                     
                end
                % 更新占用节点
                path_temp=zeros(RobotNum,1);
                path_num_temp=zeros(RobotNum,2);
                for rob = 1:RobotNum
                    path_temp(rob) = Path_num{rob,1}(res); % 储存下一节点
                    path_num_temp(rob,1)=PathStore{rob,1}(res,1);
                    path_num_temp(rob,2)=PathStore{rob,1}(res,2);
                end
                
            end
        end

        %% 直接冲突
       
            
     for l =1:RobotNum
         for p = 1:RobotNum
                temp=D;          
                for i = 1:RobotNum
                    temp(PathStore{i,1}(res,1),PathStore{i,1}(res,2)) = 1; % 将动态地图中所有机器人下一时刻所在的节点定为障碍物
                end
             
        if (path_num_temp(l,1)==path_num_temp(p,1))&&(path_num_temp(l,2)==path_num_temp(p,2))&&l~=p
                text=' 号机器人出现直接冲突，正在寻找替代路径...';
                disp([num2str(p),text]);
                if Start(p) == Goal(p)||Start(l) == Goal(l)
                    text=' 号机器人已到达终点，冲突忽略';
                    disp([num2str(p),text]);  
                    continue; %% 忽略与已经达到终点的机器人发生冲突的情况。
                end
                
                % 使用备用终点，防止终点被占据
                [X_fin,Y_fin] = spread_sin(Goal(p),SD);
                [X_start,Y_start] = spread_sin(Start(p),SD);
                
                % 暂停行动，防止起点被占据
                if temp(X_start,Y_start)==1
                    disp('起点被占用，现时刻暂停');
                    [PathStore{p,1},Path_num{p,1}]=traite_pause(D,PathStore{p,1},Path_num{p,1},res,1);
                             
                end
%                 
                
                 if X_start > encarde && Y_start > encarde && X_start+encarde <= SD && Y_start+encarde <= SD
                             disp('横纵坐标满足大于边框的要求')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                disp('终点在框外')
                                [PathStore{p,1},Path_num{p,1},Start(p),Goal(p),~]=op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start(p),Goal(p),Path_num{p,1},PathStore{p,1},p,res,SD,encarde);
                              elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                disp('终点在框内')
                               % [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp] = op_modify_sup(temp,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),res,robot_coli(j),SD,D);
                                [PathStore{p,1},Path_num{p,1},~] = ori_path_op(D,temp,X_start,Y_start,X_fin,Y_fin,PathStore{p,1},Path_num{p,1},SD,p,encarde,res);
                             else
                                 disp('终点在框外')
                                 [PathStore{p,1},Path_num{p,1},Start(p),Goal(p),~]=op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start(p),Goal(p),Path_num{p,1},PathStore{p,1},p,res,SD,encarde);
                             end
                            
                 else
                      
                             one_add_y=ones(SD,encarde);
                             one_add_x=ones(encarde,SD+2*encarde);
                             
                             D_am=D;
                             temp_am=temp;
                             
                             temp_am_op_1=[one_add_y temp_am one_add_y];
                          %   disp(temp_am_op_1)
                             temp_am_op=[one_add_x;temp_am_op_1;one_add_x];
                             
                             D_am_op_1=[one_add_y D_am one_add_y];
                             D_am_op=[one_add_x;D_am_op_1;one_add_x];
                             
                       %      [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
                             
                              disp('环境图进行扩展处理')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                 disp('终点在框外')
                                [PathStore{p,1},Path_num{p,1},Start(j),Goal(p)]=op_modify_path_am(temp,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(p),Goal(p),Path_num{p,1},PathStore{p,1},p,res,SD+2*encarde,SD,encarde);
                             elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                 disp('终点在框内') %% 可以尝试启发式方法，将求解范围缩小，修改temp，借鉴原始路径的启发式方法
                                 [PathStore{p,1},Path_num{p,1}] = ori_path_op_am(D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,PathStore{p,1},Path_num{p,1},SD,p,encarde,res);

                             else
                                 disp('终点在框外') 
                                  [PathStore{p,1},Path_num{p,1},Start(p),Goal(p)]=op_modify_path_am(temp,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(p),Goal(p),Path_num{p,1},PathStore{p,1},p,res,SD+2*encarde,SD,encarde);
                             end
                 end       
                
        end
                path_temp=zeros(RobotNum,1);
                path_num_temp=zeros(RobotNum,2);
                for rob = 1:RobotNum
                    path_temp(rob) = Path_num{rob,1}(res); % 储存下一节点
                    path_num_temp(rob,1)=PathStore{rob,1}(res,1);
                    path_num_temp(rob,2)=PathStore{rob,1}(res,2);
                end
         end
     end
        
        
        %% 节点释放
        % 判断是否求解结束
        
        MAX = 0;
         
        for i = 1:RobotNum
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
        
        if isequal(Start,Goal)
             disp('路径修改全部完成')
             flag = 1;
        else
            disp('继续')
        end
        
        
        N_F=0;
        pass=[];
        
        for i = 1:RobotNum
            if Path_num{i,1}(res)==Goal(i) && ~ismember(i,pass)
                N_F=N_F+1;
                pass=[pass i];
%                 disp(pass)
                
            end
        end
        
        flag_fin=0;
        if N_F==RobotNum
            flag=1;
            flag_fin=1;
            disp('所有机器人已经到达终点')
        end
        
%         if Path_num{i,1}(j)
    else
        flag=1;
    end
    
end

%% 补充规划变更过的终点
if flag_fin ~= 1
        inte=Goal-Start;
        for i = 1:RobotNum
            if inte(i) ~= 0
                disp('开始规划剩余路径');

          %      disp(Start(i))
         %       disp(Goal(i))
                [PATH_sup,Path_num_sup] = ori_path_am_sin(Path_num{i,1}(end),Goal(i),1,D);
          %      [PATH_sup,Path_num_sup] = ori_path(D,Start(i),Goal(i),SD,i);
         %       disp(PATH_sup)
          %      disp(Path_num_sup)
                PATH_sup(size(PATH_sup,1),:)=[];
                Path_num_sup(size(Path_num_sup,2))=[];
                PathStore{i,1}=[PathStore{i,1};PATH_sup];
                Path_num{i,1}=[Path_num{i,1} Path_num_sup];
           %     disp(PathStore{i,1})
                text = ' 号机器人所有无碰撞路径已规划完成';
                disp([num2str(i),text]);
            else
                text = ' 号机器人所有无碰撞路径已规划完成';
                disp([num2str(i),text]);
            end

        end

        MAX=0;
        for i = 1:RobotNum
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


        disp('纠正路径误差')

        for n =1:RobotNum
            [PathStore{n,1}(:,1),PathStore{n,1}(:,2)]=spread(Path_num{n,1},SD);
        end

        %% 整理路径 消除卡顿
        disp('调节路径整理参数，消除路径中不必要的卡顿')

        LEN=size(PathStore{1,1},1);
        ro=zeros(RobotNum,1);

        for n = 1:RobotNum
            if Path_num{n,1}(LEN-control)~=Goal(n) % 可以调节路径整理调整参数
                ro(n)=1;
            end
        end

        ROB=zeros(RobotNum,1);

        for n = 1:RobotNum
            if ro(n) ~=1
                flag=find(Path_num{n,1}==Goal(n));
                ROB(n)=flag(1); %第一次到达终点的索引
            end
        end

        ROB_MAX=max(ROB);

            if ismember(1,ro)
                for n =1:RobotNum
                    if ro(n) == 1
                        disp('重新规划出现卡顿的机器人')
                        [~,sup] = ori_path_am_sin(Path_num{n,1}(ROB_MAX),Goal(n),1,D);
                   %     [~,path_num_2] = ori_path_am_sin(Start_MI,Goal(n),1,D);
%                         path_num_2(1)=[];
%                         sup= [path_num_1 path_num_2];
                        if ~isempty(sup)
                            Path_num{n,1}(ROB_MAX+1:size(Path_num{n,1},2))=[];
                            Path_num{n,1}=[Path_num{n,1} sup];

                            PathStore{n,1}(size(Path_num{n,1},2)+1:size(PathStore{n,1},1),:)=[];

                            [PathStore_n,PathStore_n_i]=spread(Path_num{n,1},SD);
                            PathStore{n,1}(:,1)=PathStore_n';
                            PathStore{n,1}(:,2)=PathStore_n_i';
                        end
                    end
                end
            end

        % 统一化维度
        MAX=0;
        for i = 1:RobotNum
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
end

save('Path_num_test.mat');
% save('PathStore.mat');
