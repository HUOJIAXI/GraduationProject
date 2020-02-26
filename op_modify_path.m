%% 去重复问题
function [PathStore_MAJ_res,Path_num_MAJ_res,Start,Goal,temp]=op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start,Goal,Path_num,PathStore,j,res,SD,encarde)
    temp_reduit=temp(X_start-encarde:X_start+encarde,Y_start-encarde:Y_start+encarde); % 分割出以实际节点为中心的7*7的正方形区域，起始点为分割后的中心点13，终点为分割后子图与原路径的交点
    D_reduit=D(X_start-encarde:X_start+encarde,Y_start-encarde:Y_start+encarde);

    Goal_res_x_1=[];
    Goal_res_x_2=[];
    Goal_res_y_1=[];
    Goal_res_y_2=[];

    for k = X_start-encarde:X_start+encarde
     num_temp=Y_start-encarde+(k-1)*SD;
     if ismember(num_temp,Path_num)
         Goal_res_x_1=[Goal_res_x_1 k];  % 还需要判断是离终点最近的交点，可以通过原始的起点和终点的对应方向进行判断，但是并不准确，可以判断交点与终点的绝对距离。
     end

     num_temp=Y_start+encarde+(k-1)*SD;
     if ismember(num_temp,Path_num)
         Goal_res_x_2=[Goal_res_x_2 k]; 
     end
    end

    for k = Y_start-encarde:Y_start+encarde
     num_temp=k+(X_start-encarde-1)*SD;
     if ismember(num_temp,Path_num)
         Goal_res_y_1=[Goal_res_y_1 k];  % 还需要判断是离终点最近的交点，可以通过原始的起点和终点的对应方向进行判断，但是并不准确，可以判断交点与终点的绝对距离。
     end
     num_temp=k+(X_start+encarde-1)*SD;
     if ismember(num_temp,Path_num)
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
     SD_temp=size(temp_reduit,1);
     if temp_reduit(encarde+1,encarde+1)==1
         disp('缩减图中起点被占用，释放起点');
         temp_reduit(encarde+1,encarde+1) = 0;
     end
%      if temp_reduit(Goal_X_fin,Goal_Y_fin) == 1
%         disp('缩减图中终点被占用，释放终点');
%         temp_reduit(Goal_X_fin,Goal_Y_fin) = 0;
%      end
        disp('缩减图开始求解')
        [RE,PATH,~]=Modify_path(temp_reduit,centrale,Goal_res,j);  % 第j个冲突机器人路径重新规划，将规划得到的路径替换掉在原始路径中对应的部分。
        
         if RE == 1
          disp('终点被包围，启用备用终点,重新规划路径');
          [Goal_res,flgn,m]=GOAL_RESERVE(temp_reduit,Goal_res,SD_temp);

          if flgn == 0
             disp('环境密度太大，无法找到备用终点，求解错误');
             return;
          end

        [RE,PATH,~]=Modify_path(temp_reduit,centrale,Goal_res,j);
        
        
         if RE ==0
             disp('备用终点启用成功');
              if flgn==1
                    disp('返回原终点-m')
                    disp('释放原始终点')
                    temp_reduit(Goal_X_fin,Goal_Y_fin)=0;
                    Goal_res=Goal_res-m; % 返回原始终点
                    [RE,PATH_sup,~]=sup_path(temp_reduit,D_reduit,Goal_res+m,Goal_res,SD_temp,j) ; %需要扣除重复的点
     %               disp(PATH_sup)
                    if m ~= 1
                        PATH(size(PATH,1),:)=[];
                    end
                    disp(PATH)
                    PATH=[PATH;PATH_sup];
        %                     Path_num_MAJ=[Path_num_MAJ path_num_sup];
                    disp(PATH)
                    if RE == 0
                        disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                    end

              end

              if flgn==2
                    disp('返回原终点+m')
                    Goal_res=Goal_res+m;
                    disp('释放原始终点')
                    temp_reduit(Goal_X_fin,Goal_Y_fin)=0;
                    if m ~= 1
                        PATH(size(PATH,1),:)=[];
                    end
                    [RE,PATH_sup,~]=sup_path(temp_reduit,D_reduit,Goal_res-m,Goal_res,SD_temp,j)  ;
                    PATH=[PATH;PATH_sup];
        %                     Path_num_MAJ=[Path_num_MAJ path_num_sup];
                    if RE == 0
                        disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                    end
              end
         end
         
         end
         
         if RE == 1
  
             disp('该备用终点依然被包围，道路被完全阻挡')

             disp('寻找备用终点失败，机器人在此时刻暂停行动，或是已经到达终点')
             
            PATH= PathStore(res-1:size(PathStore,1),:); % 暂停有问题
            Path_num_MAJ=Path_num(res-1:size(Path_num,2));

            PathStore(res:size(PathStore,1),:)=[];
            Path_num(res:size(Path_num,2))=[];

            PathStore_MAJ_res=[PathStore;PATH];
            Path_num_MAJ_res=[Path_num Path_num_MAJ];
                
            return
%              PathStore_MAJ_res= [PathStore(res-1,:) ; PathStore(res-1:size(PathStore,1),:)];
%              Path_num_MAJ_res=(PathStore_MAJ_res(:,2)+(PathStore_MAJ_res(:,1)-1)*SD)';
             
         else

            PATH(:,1)=PATH(:,1)+X_start-encarde-1;
            PATH(:,2)=PATH(:,2)+Y_start-encarde-1;
            Path_num_MAJ=(PATH(:,2)+(PATH(:,1)-1)*SD)'; %  转换坐标


            z=find(Path_num==Goal_res_temp); % 寻找最后一个标号的位置

            PathStore_temp=PathStore(z+1:size(PathStore,1),:);
            Path_num_temp=Path_num(z+1:size(Path_num,2));

            PathStore(res-1:size(PathStore,1),:)=[];
            Path_num(res-1:size(Path_num,2))=[];

            PathStore_MAJ_res=[PathStore;PATH;PathStore_temp];
            Path_num_MAJ_res=[Path_num,Path_num_MAJ,Path_num_temp]; % 替换矩阵部分
         end
        
    else
         [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start,Goal,j);  % 第j个冲突机器人路径重新规划
 %   toc

        if RE == 1
              disp('终点被包围，启用备用终点,重新规划路径');
              [Goal,flgn,m]=GOAL_RESERVE(temp,Goal,SD);

          if flgn == 0
             disp('环境密度太大，无法找到备用终点，求解错误');
             return;
          end

          [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start,Goal,j);
          
           if RE == 0
               disp('备用终点启用成功');     
              if flgn==1
                    disp('返回原终点-2')
                    Goal=Goal-m; % 返回原始终点
                    if m ~= 1
                        PATH(size(PATH,1),:)=[];
                    end
                    [PATH_sup,path_num_sup]=sup_path_ori(temp,Goal+m,Goal,SD,j)  ;
                    PATH=[PATH;PATH_sup];
                    Path_num_MAJ=[Path_num_MAJ path_num_sup];
                    if RE == 0
                        disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                    end

              end

              if flgn==2
                    disp('返回原终点+2')
                    Goal=Goal+m;
                    if m ~= 1
                        PATH(size(PATH,1),:)=[];
                    end
                    [PATH_sup,path_num_sup]=sup_path_ori(temp,Goal-m,Goal,SD,j)  ;
                    PATH=[PATH;PATH_sup];
                    Path_num_MAJ=[Path_num_MAJ path_num_sup];
                    if RE == 0
                        disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                    end
              end
           
          else
             if flgn==1
                    disp('返回原终点-2')
                    Goal=Goal-m; % 返回原始终点
              end

              if flgn==2
                    disp('返回原终点+2')
                    Goal=Goal+m;
              end
       %      [X_fin_it,Y_fin_it] = spread_sin(Goal,SD);
                 disp('该备用终点依然被包围，道路被完全阻挡')
          %   temp_ori(X_fin_it,Y_fin_it)=1; % 将无法使用的备用终点排除
                 disp('寻找备用终点失败，机器人暂停在此时刻暂停行动')
                % n_robot=find(Path_num==Start);
%                 PATH= [PathStore(res-1,:) ; PathStore(res-1:size(PathStore,1),:)]; % 暂停有问题
%                 Path_num_MAJ=[Path_num(res-1),Path_num(res-1:size(Path_num,2))];
                
                PATH= PathStore(res-1:size(PathStore,1),:); % 暂停有问题
                Path_num_MAJ=Path_num(res-1:size(Path_num,2));
                
                PathStore(res:size(PathStore,1),:)=[];
                Path_num(res:size(Path_num,2))=[];

                PathStore_MAJ_res=[PathStore;PATH];
                Path_num_MAJ_res=[Path_num Path_num_MAJ];
                return
                
          end

        end

         

        PathStore(res-1:size(PathStore,1),:)=[];
        Path_num(res-1:size(Path_num,2))=[];

        PathStore_MAJ_res=[PathStore;PATH];
        Path_num_MAJ_res=[Path_num Path_num_MAJ];
    end
