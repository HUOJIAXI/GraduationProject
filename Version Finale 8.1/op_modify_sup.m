%% 版本6.1探索应用分割法

function [PathStore_MAJ_Res,Path_num_MAJ_Res,Start,Goal,temp] = op_modify_sup(temp,PathStore,Path_num,Start,Goal,res,j,SD,D)
    
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

          if flgn==1
                Goal=Goal-m; % 返回原始终点
                [RE,PATH_sup,path_num_sup]=sup_path(D,D,Goal+m,Goal,SD,j)  ;
                    if (PATH_sup(1,1)-PATH(end,1)==0 && PATH_sup(1,2)-PATH(end,2)==0)
                        PATH(size(PATH,1),:)=[];
                    end
                PATH=[PATH;PATH_sup];
                Path_num_MAJ=[Path_num_MAJ path_num_sup];
                if RE == 0
                    disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                end
          end

          if flgn==2
                Goal=Goal+m;
                [RE,PATH_sup,path_num_sup]=sup_path(D,D,Goal-m,Goal,SD,j)  ;
                PATH=[PATH;PATH_sup];
                    if (PATH_sup(1,1)-PATH(end,1)==0 && PATH_sup(1,2)-PATH(end,2)==0)
                        PATH(size(PATH,1),:)=[];
                    end
                Path_num_MAJ=[Path_num_MAJ path_num_sup];
                if RE == 0
                    disp('备用终点启用成功，已生成备用路径，已切换回原始终点');     
                end
          end
    end

     if RE == 1
         disp('该备用终点依然被包围，道路被完全阻挡')
         disp('寻找备用终点失败，机器人暂停在此时刻暂停行动')
                % n_robot=find(Path_num==Start);
        [PathStore_MAJ_Res,Path_num_MAJ_Res]=traite_pause(D,PathStore,Path_num,res,1);
        
        return
%         PATH= [PathStore(res-1,:) ; PathStore(res-1:size(PathStore,1),:)];
%         Path_num_MAJ=[Path_num(res-1),Path_num(res-1:size(Path_num,2))];
     end

    PathStore(res-1:size(PathStore,1),:)=[];
    Path_num(res-1:size(Path_num,2))=[];

    PathStore_MAJ_Res=[PathStore;PATH];
    Path_num_MAJ_Res=[Path_num Path_num_MAJ];