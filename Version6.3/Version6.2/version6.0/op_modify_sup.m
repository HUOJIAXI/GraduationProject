function [PathStore_MAJ_Res,Path_num_MAJ_Res,Start,Goal,temp] = op_modify_sup(temp,PathStore,Path_num,Start,Goal,res,j,SD,D)
    
    [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start,Goal,j);  % 第j个冲突机器人路径重新规划
    %   toc

    temp_ori =temp;

    while 1
        if RE == 0
            break
        end

        if RE == 1
              disp('终点被包围，启用备用终点,重新规划路径');
              [Goal,flgn,m]=GOAL_RESERVE(temp_ori,Goal,SD);

              if flgn == 0
                 disp('环境密度太大，无法找到备用终点，求解错误');
                 return;
              end

              [RE,PATH,Path_num_MAJ]=Modify_path(temp,Start,Goal,j);

              if flgn==1
                    Goal=Goal-m; % 返回原始终点
                    [PATH_sup,path_num_sup]=sup_path(D,Goal+m,Goal,SD,j)  ;
                    PATH=[PATH;PATH_sup];
                    Path_num_MAJ=[Path_num_MAJ path_num_sup];
                    if RE == 0
                        disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                        break
                    end
              end

              if flgn==2
                    Goal=Goal+m;
                    [PATH_sup,path_num_sup]=sup_path(D,Goal-m,Goal,SD,j)  ;
                    PATH=[PATH;PATH_sup];
                    Path_num_MAJ=[Path_num_MAJ path_num_sup];
                    if RE == 0
                        disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                        break
                    end
              end
        end

         if RE == 1
             [X_fin_it,Y_fin_it] = spread_sin(Goal,SD);
             disp('该备用终点依然被包围，重新寻找备用终点')
             temp_ori(X_fin_it,Y_fin_it)=1; % 将无法使用的备用终点排除
         end

    end

    PathStore(res-1:size(PathStore,1),:)=[];
    Path_num(res-1:size(Path_num,2))=[];

    PathStore_MAJ_Res=[PathStore;PATH];
    Path_num_MAJ_Res=[Path_num Path_num_MAJ];