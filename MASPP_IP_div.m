%% 本版本基于MASPP_IP，探究通过子图分割降低路径修改时延

function [PathStore,Path_num] = MASPP_IP_div(D,RobotNum,Start,Goal)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;

SD=size(D,1);
Goal_ori=Goal;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
for i = 1:RobotNum
    [X_start,Y_start] = spread_sin(Start(i),SD);
    [X_fin,Y_fin] = spread_sin(Goal(i),SD);
    squ=max(abs(X_start-X_fin),abs(Y_start-Y_fin));
    ini_x=min(X_start,X_fin);
    ini_y=min(Y_start,Y_fin);
    if (ini_x+squ)<=SD && (ini_y+squ)<=SD
        
        Start_op_x=X_start-ini_x+1;
        Start_op_y=Y_start-ini_y+1;
        Goal_op_x =X_fin  -ini_x+1;
        Goal_op_y =Y_fin  -ini_y+1;
        Start_op=Start_op_y+(Start_op_x-1)*(squ+1);
        Goal_op = Goal_op_y+(Goal_op_x-1)*(squ+1);
        
        temp_D=D(ini_x:ini_x+squ,ini_y:ini_y+squ);
        [PATH,~]=IP_solver(temp_D,Start_op,Goal_op,i);
        PATH(:,1)=PATH(:,1)+ini_x-1;
        PATH(:,2)=PATH(:,2)+ini_y-1;
        %path_num=path_num+(ini_y-1)+(ini_x-1)*SD;
        path_num=(PATH(:,2)+(PATH(:,1)-1)*SD)';
    else
        [PATH,path_num]=IP_solver(D,Start(i),Goal(i),i);
    end
    
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

    %% 2020年2月12日版本，需进一步修改。
                     Goal_res_x_1=[];
                     Goal_res_x_2=[];
                     Goal_res_y_1=[];
                     Goal_res_y_2=[];
                     encarde = 2;
                     if X_start > encarde && Y_start > encarde
                             disp('横纵坐标满足大于encarde的要求')
                             %% 优化碰撞处理
                             %if (Y_fin-Y_start)*(Y_fin-Y_start)+ (X_fin-X_start)*(X_fin-X_start)>= (encarde+1)*(encarde+1)*2
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                 disp('终点在框外')
                                 temp_reduit=temp(X_start-encarde:X_start+encarde,Y_start-encarde:Y_start+encarde); % 分割出以实际节点为中心的7*7的正方形区域，起始点为分割后的中心点13，终点为分割后子图与原路径的交点

                                 for k = X_start-encarde:X_start+encarde
                                     num_temp=Y_start-encarde+(k-1)*SD;
                                     if ismember(num_temp,Path_num{i,1})
                                         Goal_res_x_1=[Goal_res_x_1 k];  % 还需要判断是离终点最近的交点，可以通过原始的起点和终点的对应方向进行判断，但是并不准确，可以判断交点与终点的绝对距离。
                                     end
                                     
                                     num_temp=Y_start+encarde+(k-1)*SD;
                                     if ismember(num_temp,Path_num{i,1})
                                         Goal_res_x_2=[Goal_res_x_2 k]; 
                                     end
                                 end

                                 for k = Y_start-encarde:Y_start+encarde
                                     num_temp=k+(X_start-encarde-1)*SD;
                                     if ismember(num_temp,Path_num{i,1})
                                         Goal_res_y_1=[Goal_res_y_1 k];  % 还需要判断是离终点最近的交点，可以通过原始的起点和终点的对应方向进行判断，但是并不准确，可以判断交点与终点的绝对距离。
                                     end
                                     num_temp=k+(X_start+encarde-1)*SD;
                                     if ismember(num_temp,Path_num{i,1})
                                         Goal_res_y_2=[Goal_res_y_2 k]; 
                                     end
                                 end

                                 short_1=[];
                                 short_2=[];
                                 short_3=[];
                                 short_4=[];

                                 if(~isempty(Goal_res_x_1))
                                     for m = 1:length(Goal_res_x_1)
                                         short_1(m,1) = (Goal_res_x_1(m)-X_fin)*(Goal_res_x_1(m)-X_fin)+(Y_fin-Y_start+encarde)*(Y_fin-Y_start+encarde);
                                         short_1(m,2) = Goal_res_x_1(m);
                                     end
                                 end

                                 if(~isempty(Goal_res_x_2))
                                     for m = 1:length(Goal_res_x_2)
                                         short_2(m,1) = (Goal_res_x_2(m)-X_fin)*(Goal_res_x_2(m)-X_fin)+(Y_fin-Y_start-encarde)*(Y_fin-Y_start-encarde);
                                         short_2(m,2) = Goal_res_x_2(m);
                                     end
                                 end

                                 if(~isempty(Goal_res_y_1))
                                     for m = 1:length(Goal_res_y_1)
                                         short_3(m,1) = (Goal_res_y_1(m)-Y_fin)*(Goal_res_y_1(m)-Y_fin)+(X_fin-X_start+encarde)*(X_fin-X_start+encarde);
                                         short_3(m,2) = Goal_res_y_1(m);
                                     end
                                 end

                                 if(~isempty(Goal_res_y_2))
                                     for m = 1:length(Goal_res_y_2)
                                         short_4(m,1) = (Goal_res_y_2(m)-Y_fin)*(Goal_res_y_2(m)-Y_fin)+(X_fin-X_start-encarde)*(X_fin-X_start-encarde);
                                         short_4(m,2) = Goal_res_y_2(m); % temp
                                     end
                                 end

                                 if (~isempty(Goal_res_y_2)||~isempty(Goal_res_y_1)||~isempty(Goal_res_x_2)||~isempty(Goal_res_x_1))
                                     if(~isempty(short_1)) 
                                         [shorest(1,1),p]=min(short_1(:,1));
                                         shorest(1,2)=short_1(p,2);
                                     else
                                         shorest(1,1)=10000;
                                         shorest(1,2)=100;
                                     end

                                     if(~isempty(short_2))
                                         [shorest(2,1),p]=min(short_2(:,1));
                                         shorest(2,2)=short_2(p,2);
                                     else
                                         shorest(2,1)=10000;
                                         shorest(2,2)=100;
                                     end
                                     if(~isempty(short_3))
                                         [shorest(3,1),p]=min(short_3(:,1));
                                         shorest(3,2)=short_3(p,2);
                                     else
                                         shorest(3,1)=10000;
                                         shorest(3,2)=100;
                                     end
                                     if(~isempty(short_4))
                                         [shorest(4,1),p]=min(short_4(:,1));
                                         shorest(4,2)=short_4(p,2);
                                     else
                                         shorest(4,1)=10000;
                                         shorest(4,2)=100;
                                     end

                                     [~,f]=min(shorest(:,1));

                                    % if (Y_fin-Y_start)*(Y_fin-Y_start)+ (X_fin-X_start)*(X_fin-X_start)> (encarde+1)*(encarde+1)*2
                                         if f == 1
                                             Goal_res_x=shorest(1,2);
                                             Goal_res_y=Y_start-encarde;
                                         elseif f == 2
                                             Goal_res_x=shorest(2,2);
                                             Goal_res_y=Y_start+encarde;
                                         elseif f == 3
                                             Goal_res_y=shorest(3,2);
                                             Goal_res_x=X_start-encarde;
                                         elseif f == 4
                                             Goal_res_y=shorest(4,2);
                                             Goal_res_x=X_start+encarde;
                                         end

                                         %Goal_res = (Goal_res_x-X_start+3+1)+(Goal_res_y-Y_start+2)*7; %转换为temp_reduit中的标号
                                     %if shorest_path > (encarde+1)*(encarde+1)*2
                                         Goal_res_temp=Goal_res_y+(Goal_res_x-1)*SD;
                                         encarde_total=encarde*2+1;
                                         Goal_res = (Goal_res_y-Y_start+encarde+1)+(Goal_res_x-(X_start-encarde))*encarde_total; %转换为temp_reduit中的标号

                                         centrale=encarde_total*encarde+encarde+1;
                                         %   tic
                                         [Goal_X_fin,Goal_Y_fin] = spread_sin(Goal_res,encarde_total);
                                         if temp_reduit(encarde+1,encarde+1)==1
                                             disp('缩减图中起点被占用，释放起点');
                                             temp_reduit(encarde+1,encarde+1) = 0;
                                         end
                                         if temp_reduit(Goal_X_fin,Goal_Y_fin) == 1
                                            disp('缩减图中终点被占用，释放终点');
                                            temp_reduit(Goal_X_fin,Goal_Y_fin) = 0;
                                         end
                                            disp('缩减图开始求解')
                                            [~,PATH,Path_num_MAJ]=Modify_path(temp_reduit,centrale,Goal_res,j);  % 第j个冲突机器人路径重新规划，将规划得到的路径替换掉在原始路径中对应的部分。

                                            %u = length(Path_num_MAJ); % 最后一个标号
                                             %   toc
                                            PATH(:,1)=PATH(:,1)+X_start-encarde-1;
                                            PATH(:,2)=PATH(:,2)+Y_start-encarde-1;
                                            Path_num_MAJ=(PATH(:,2)+(PATH(:,1)-1)*SD)'; %  转换坐标


                                            z=find(Path_num{j,1}==Goal_res_temp); % 寻找最后一个标号的位置

                                            PathStore_temp=PathStore{j,1}(z+1:size(PathStore{j,1},1),:);
                                            Path_num_temp=Path_num{j,1}(z+1:size(Path_num{j,1},2));

                                            PathStore{j,1}(res-1:size(PathStore{j,1},1),:)=[];
                                            Path_num{j,1}(res-1:size(Path_num{j,1},2))=[];

                                            PathStore{j,1}=[PathStore{j,1};PATH;PathStore_temp];
                                            Path_num{j,1}=[Path_num{j,1},Path_num_MAJ,Path_num_temp]; % 替换矩阵部分
                                     else
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
                                                        disp('返回原终点-2')
                                                        Goal(j)=Goal(j)-m; % 返回原始终点
                                                        [PATH_sup,path_num_sup]=sup_path(D,Goal(j)+m,Goal(j),SD,j)  ;
                                                        PATH=[PATH;PATH_sup];
                                                        Path_num_MAJ=[Path_num_MAJ path_num_sup];
                                                        if RE == 0
                                                            disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                                                        break
                                                        end
                                                  
                                                  end

                                                  if flgn==2
                                                        disp('返回原终点+2')
                                                        Goal(j)=Goal(j)+m;
                                                        [PATH_sup,path_num_sup]=sup_path(D,Goal(j)-m,Goal(j),SD,j)  ;
                                                        PATH=[PATH;PATH_sup];
                                                        Path_num_MAJ=[Path_num_MAJ path_num_sup];
                                                        if RE == 0
                                                            disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                                                        break
                                                        end
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

                                 else
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
                                                      Goal(j)=Goal(j)-m; 
                                                      [PATH_sup,path_num_sup]=sup_path(D,Goal(j)+m,Goal(j),SD,j)  ;
                                                      PATH=[PATH;PATH_sup];
                                                      Path_num_MAJ=[Path_num_MAJ path_num_sup];
                                                      % 返回原始终点
                                                        if RE == 0
                                                            disp('备用终点启用成功，已生成备用路径，已切换回原始终点');                                           
                                                            break
                                                        end
                                                  end

                                                  if flgn==2
                                                        Goal(j)=Goal(j)+m;
                                                        [PATH_sup,path_num_sup]=sup_path(D,Goal(j)-m,Goal(j),SD,j)  ;
                                                        PATH=[PATH;PATH_sup];
                                                        Path_num_MAJ=[Path_num_MAJ path_num_sup];
                                                        if RE == 0
                                                            disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                                                            break
                                                        end
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
                            %% 
                     else
                         %% 补充原始节点的横纵坐标小于等于3的情况
                         
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
                                            [PATH_sup,path_num_sup]=sup_path(D,Goal(j)+m,Goal(j),SD,j)  ;
                                            PATH=[PATH;PATH_sup];
                                            Path_num_MAJ=[Path_num_MAJ path_num_sup];
                                            if RE == 0
                                                disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                                                break
                                            end
                                      end

                                      if flgn==2
                                            Goal(j)=Goal(j)+m;
                                            [PATH_sup,path_num_sup]=sup_path(D,Goal(j)-m,Goal(j),SD,j)  ;
                                            PATH=[PATH;PATH_sup];
                                            Path_num_MAJ=[Path_num_MAJ path_num_sup];
                                            if RE == 0
                                                disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                                                break
                                            end
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
                     
                [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(robot_coli(j)),Goal(robot_coli(j)),robot_coli(j));  % 第j个冲突机器人路径重新规划
                
                temp_ori =temp;
                while 1
                    if RE == 0
                           break
                    end
                        
                    if RE == 1
                          disp('终点被包围，启用备用终点,重新规划路径');
                          [Goal(robot_coli(j)),flgn,m]=GOAL_RESERVE(temp_ori,Goal(robot_coli(j)),SD);

                          if flgn == 0
                             disp('环境密度太大，无法找到备用终点，求解错误');
                             return;
                          end

                          [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start(robot_coli(j)),Goal(robot_coli(j)),robot_coli(j));
                          
                          if flgn==1
                            Goal(robot_coli(j))=Goal(robot_coli(j))-m; % 返回原始终点
                            [PATH_sup,path_num_sup]=sup_path(D,Goal(robot_coli(j))+m,Goal(robot_coli(j)),SD,j)  ;
                            PATH=[PATH;PATH_sup];
                            Path_num_MAJ=[Path_num_MAJ path_num_sup];
                          end

                          if flgn==2
                             Goal(robot_coli(j))=Goal(robot_coli(j))+m;
                            [PATH_sup,path_num_sup]=sup_path(D,Goal(robot_coli(j))-m,Goal(robot_coli(j)),SD,j)  ;
                            PATH=[PATH;PATH_sup];
                            Path_num_MAJ=[Path_num_MAJ path_num_sup];
                          end
                    end
                   
                    
                     if RE == 0
                            disp('备用终点启用成功，已生成备用路径，已切换为原始终点');     
                            break
                     end

                     if RE == 1
                         [X_fin_it,Y_fin_it] = spread_sin(Goal(robot_coli(j)),SD);
                         disp('该备用终点依然被包围，重新寻找备用终点')
                         temp_ori(X_fin_it,Y_fin_it)=1; % 将无法使用的备用终点排除
                     end
                 
                end
                            

                PathStore{robot_coli(j),1}(res-1:size(PathStore{robot_coli(j),1},1),:)=[];
                Path_num{robot_coli(j),1}(res-1:size(Path_num{robot_coli(j),1},2))=[];
                
                PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1};PATH];
                Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ]; 
                
%                 if Fn == 1
%                     temp(X_fin,Y_fin)=1; % 恢复被清空的终点
%                 end
                
            end
        end
        
        %% 节点释放

        for i = 1:RobotNum
            temp(PathStore{i}(res,1),PathStore{i}(res,2))=0; % 释放当前节点
        end
        
        SAME=unique(Start-Goal);
        same_num=find(SAME);
        
        if isequal(path_temp,Goal) || SAME(same_num(1))==2 || SAME(same_num(1))==-2
        %if isequal(path_temp,Goal)
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
load('Path_num.mat');




