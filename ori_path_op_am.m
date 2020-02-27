function [ PathStore_MAJ_res,Path_num_MAJ_res] = ori_path_op_am(D_ori,D,X_start,Y_start,X_fin,Y_fin,PathStore,Path_num,SD,i,encarde,res)

    Goal_op_x=X_fin-X_start+encarde+1; % 终点坐标转换
    Goal_op_y=Y_fin-Y_start+encarde+1;
    Goal_op = Goal_op_y+(Goal_op_x-1)*(2*encarde+1);
    temp_D=D(X_start+encarde-encarde:X_start+encarde+encarde,Y_start+encarde-encarde:Y_start+encarde+encarde);
    D_reduit=D_ori(X_start+encarde-encarde:X_start+encarde+encarde,Y_start+encarde-encarde:Y_start+encarde+encarde);
    SD_temp=size(temp_D,1);
    central=SD_temp*encarde+encarde+1;
    [RE,PATH,~]=Modify_path(temp_D,central,Goal_op,i);
    
      if RE == 1
          disp('终点被包围，启用备用终点,重新规划路径');
          [Goal_op,flgn,m]=GOAL_RESERVE(temp_D,Goal_op,SD_temp);

          if flgn == 0
             disp('环境密度太大，无法找到备用终点，求解错误');
             return;
          end

          [RE,PATH,~]=Modify_path(temp_D,central,Goal_op,i);
          
          
              if RE == 0
              disp('备用终点启用成功，已生成备用路径，已切换回原始终点'); 
                  if flgn==1
                        disp('返回原终点-m')
                     %   disp(temp_D(Goal_op_x,Goal_op_y));
                        disp('释放原始终点')
                        temp_D(Goal_op_x,Goal_op_y)=0;
                        Goal_op=Goal_op-m; % 返回原始终点
                        if m ~= 1
                        PATH(size(PATH,1),:)=[];
                        end
                        [RE,PATH_sup,~]=sup_path(temp_D,D_reduit,Goal_op+m,Goal_op,SD_temp,i)  ;
                        PATH=[PATH;PATH_sup];
                     %   disp(PATH)
                  end

                  if flgn==2
                        disp('返回原终点+m')
                        Goal_op=Goal_op+m;
                        disp('释放原始终点')
                        temp_D(Goal_op_x,Goal_op_y)=0;
                       if m ~= 1
                        PATH(size(PATH,1),:)=[];
                       end
                        [RE,PATH_sup,~]=sup_path(temp_D,D_reduit,Goal_op-m,Goal_op,SD_temp,i)  ;
                        PATH=[PATH;PATH_sup];
                  end
              end
     end

     if RE == 1
    %      [X_fin_it,Y_fin_it] = spread_sin(Goal,SD);
         disp('该备用终点依然被包围，道路被完全阻挡')
      %   temp_ori(X_fin_it,Y_fin_it)=1; % 将无法使用的备用终点排除
         disp('寻找备用终点失败，机器人在此时刻暂停行动')
            % n_robot=find(Path_num==Start);
         [PathStore_MAJ_res,Path_num_MAJ_res]=traite_pause(D,PathStore,Path_num,res,1);
%          PATH= [PathStore(res-1,:) ; PathStore(res-1:size(PathStore,1),:)];
%          
%          Path_num_MAJ=[Path_num(res-1),Path_num(res-1:size(Path_num,2))];
%              
%          PathStore(res-1:size(PathStore,1),:)=[];
%          Path_num(res-1:size(Path_num,2))=[];
% 
%          PathStore_MAJ_res=[PathStore;PATH];
%          Path_num_MAJ_res=[Path_num Path_num_MAJ];
         return
          %  Path_num_MAJ=[Path_num(res-1),Path_num(res-1:size(Path_num,2))];
     end

    PATH_MAJ(:,1)=PATH(:,1)+X_start-encarde-1;
    PATH_MAJ(:,2)=PATH(:,2)+Y_start-encarde-1;
    %path_num=path_num+(ini_y-1)+(ini_x-1)*SD;
 %
 %disp(PATH_MAJ)
    
    Path_num_MAJ=(PATH_MAJ(:,2)+(PATH_MAJ(:,1)-1)*SD)';

    PathStore(res-1:size(PathStore,1),:)=[];
    Path_num(res-1:size(Path_num,2))=[];

    PathStore_MAJ_res=[PathStore;PATH_MAJ];
    Path_num_MAJ_res=[Path_num Path_num_MAJ];