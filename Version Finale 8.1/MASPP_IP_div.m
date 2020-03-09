%% 本版本基于MASPP_IP，探究通过子图分割降低路径修改时延

function [PathStore,Path_num] = MASPP_IP_div(D,RobotNum,Start,Goal,encarde,control)

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

MAX=0;

SD=size(D,1);
%Goal_ori=Goal;



%% 存储单机器人的原始最佳路径
% 扩展路径矩阵，使得所有矩阵的维度一样，方便之后的去障碍算法。
for i = 1:RobotNum
    [X_ST,Y_ST]=spread(Start(i),SD);
    [X_FI,Y_FI]=spread(Goal(i),SD);
    X_MI=floor((X_ST+X_FI)/2);
    Y_MI=floor((Y_ST+Y_FI)/2);
%     disp(X_MI)
%     disp(Y_MI)
    if X_MI>=1 || Y_MI>=1
        if D(X_MI,Y_MI)==0 &&  (abs(X_MI-X_FI)>=3 || abs(Y_MI-Y_FI)>=3) && (abs(X_MI-X_ST)>=3 || abs(Y_MI-Y_ST)>=3)
            Start_MI=(Y_MI+(X_MI-1)*SD);
            [PATH_1,path_num_1] = ori_path(D,Start(i),Start_MI,SD,i);
            [PATH_2,path_num_2] = ori_path(D,Start_MI,Goal(i),SD,i);
            PATH_2(1,:)=[];
            path_num_2(1)=[];
            PathStore{i,1} = [PATH_1;PATH_2];
            Path_num{i,1} = [path_num_1 path_num_2];
        elseif D(X_MI,Y_MI)==1 && (abs(X_MI-X_FI)>=3 || abs(Y_MI-Y_FI)>=3) && (abs(X_MI-X_ST)>=3 || abs(Y_MI-Y_ST)>=3)
            Y_MI=Y_MI-1;
            Start_MI=(Y_MI+(X_MI-1)*SD);
            [PATH_1,path_num_1] = ori_path(D,Start(i),Start_MI,SD,i);
            [PATH_2,path_num_2] = ori_path(D,Start_MI,Goal(i),SD,i);
            PATH_2(1,:)=[];
            path_num_2(1)=[];
            PathStore{i,1} = [PATH_1;PATH_2];
            Path_num{i,1} = [path_num_1 path_num_2];
            
        else
            [PATH,path_num] = ori_path(D,Start(i),Goal(i),SD,i); % 将原始路径规划分块
            PathStore{i,1} = PATH;
            Path_num{i,1} = path_num;
        end
    
    else
        [PATH,path_num] = ori_path(D,Start(i),Goal(i),SD,i); % 将原始路径规划分块
        PathStore{i,1} = PATH;
        Path_num{i,1} = path_num;
    end
    
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
    if res < 25
        %% 解决两种冲突，迎面冲突和转角冲突
        for i = 1:RobotNum
            temp(PathStore{i,1}(res+1,1),PathStore{i,1}(res+1,2)) = 1; % 将动态地图中所有机器人下一时刻所在的节点定为障碍物
        %    temp(PathStore{i,1}(res,1),PathStore{i,1}(res,2)) = 1; 
            Start(i)=Path_num{i,1}(res); % 将机器人实际所在节点作为出发点
        end
       
        
        res = res + 1; % 从res时刻到下一时刻res+1
        %save('temp.mat');
 
        
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
                if (abs(path_temp(i)-path_temp(j))==SD && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==SD)||(abs(path_temp(i)-path_temp(j))==1 && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==1)
                  % (abs(path_temp(i)-path_temp(j))==SD && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==SD)||(abs(path_temp(i)-path_temp(j))==1 && abs(Path_num{i,1}(res-1)-Path_num{j,1}(res-1))==1)||  
                  %||  (abs(path_num_temp(i,2)-path_num_temp(j,2))==1 &&
                  %abs(path_num_temp(i,1)-path_num_temp(j,1))==0)||(abs(path_num_temp(i,2)-path_num_temp(j,2))==0 && abs(path_num_temp(i,1)-path_num_temp(j,1))==1)&&i~=j
                    if Start(j) == Goal(j)
                    text=' 号机器人已到达终点，冲突忽略';
                    disp([num2str(j),text]);  
                    continue; %% 忽略与已经达到终点的机器人发生冲突的情况。
                    end
                    text=' 号机器人出现交叉冲突，正在寻找替代路径...';
                    disp([num2str(j),text]);
                    [X_start,Y_start] = spread_sin(Start(j),SD);
%                     disp(num2str(temp(X_start,Y_start)))
                    disp('交叉冲突释放起始节点')
                    temp(X_start,Y_start)=0; % 释放起始节点
%                     disp(num2str(temp(X_start,Y_start)))

                     [X_fin,Y_fin] = spread_sin(Goal(j),SD);
%                     disp(num2str(temp(X_fin,Y_fin)))
%                     disp('交叉冲突临时释放目标节点')
%                     temp(X_fin,Y_fin)=0;% 释放目标节点
%                     disp(num2str(temp(X_fin,Y_fin)))

%                     encarde = 2;
                     
                     if X_start > encarde && Y_start > encarde && X_start+encarde <= SD && Y_start+encarde <= SD
                             disp('横纵坐标满足大于边框的要求')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                 disp('终点在框外')
                                [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),~]=op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD,encarde);
                             elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                 disp('终点在框内') %% 可以尝试启发式方法，将求解范围缩小，修改temp，借鉴原始路径的启发式方法
                                 [PathStore{j,1},Path_num{j,1},~] = ori_path_op(D,temp,X_start,Y_start,X_fin,Y_fin,PathStore{j,1},Path_num{j,1},SD,j,encarde,res);
%                                if RE == 1
%                                   disp('启发式算法失效')
%                                  [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
%                                end
                             else
                                 disp('终点在框外') 
                                 [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),~] = op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD,encarde);
                             end
                            
                     else
                         % 补充原始节点的横纵坐标小于等于3的情况，需要增加额外的启发式算法
                             disp('横或纵坐标不满足大于边框的要求')                           
                              % 优化边界冲突处理，将原环境拓展
                             one_add_y=ones(SD,encarde);
                             one_add_x=ones(encarde,SD+2*encarde);
                             temp_am=temp;
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
                                [PathStore{j,1},Path_num{j,1},Start(j),Goal(j)]=op_modify_path_am(temp,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD+2*encarde,SD,encarde);
                             elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                 disp('终点在框内') %% 可以尝试启发式方法，将求解范围缩小，修改temp，借鉴原始路径的启发式方法
                                 [PathStore{j,1},Path_num{j,1}] = ori_path_op_am(D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,PathStore{j,1},Path_num{j,1},SD,j,encarde,res);
%                                if RE == 1
%                                   disp('启发式算法失效')
%                                  [PathStore{j,1},Path_num{j,1},Start(j),Goal(j),temp] = op_modify_sup(temp,PathStore{j,1},Path_num{j,1},Start(j),Goal(j),res,j,SD,D);
%                                end
                             else
                                 disp('终点在框外') 
                                  [PathStore{j,1},Path_num{j,1},Start(j),Goal(j)]=op_modify_path_am(temp,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(j),Goal(j),Path_num{j,1},PathStore{j,1},j,res,SD+2*encarde,SD,encarde);
                             end
                             
                     
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
                 same_x=path_num_temp(l,1);
                 same_y=path_num_temp(l,2);
                 for p=1:RobotNum
                     if (same == path_temp(p) && l ~= p) || (same_x==path_num_temp(p,1) && same_y==path_num_temp(p,2) && l~=p)
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
                   % PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1};(PathStore{robot_coli(j),1}(res-1,:))];
                   
%                    flag_stop=0;
%                     for z=1:RobotNum
%                         if Path_num{robot_coli(j),1}(res-1) == Path_num{robot_coli(z),1}(res)
%                             flag_stop=1;
%                             break
%                         end
%                     end
                    [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1}]=traite_pause(D,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},res,1);
                    
%                     if flag_stop == 0
%                         PATH= PathStore{robot_coli(j),1}(res-1:size(PathStore{robot_coli(j),1},1),:);
%                         Path_num_MAJ=Path_num{robot_coli(j),1}(res-1:size(Path_num{robot_coli(j),1},2));
% 
%                         PathStore{robot_coli(j),1}(res:size(PathStore{robot_coli(j),1},1),:)=[];
%                         Path_num{robot_coli(j),1}(res:size(Path_num{robot_coli(j),1},2))=[];
% 
%                         PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1} ; PATH];
%                         Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ];
%                         continue
%                     else
%                         if PathStore{robot_coli(j),1}(res-1,2)-1~=0 || PathStore{robot_coli(j),1}(res-1,1)-1~=0
%                             if PathStore{robot_coli(j),1}(res-1,2)-1~=0 && D(PathStore{robot_coli(j),1}(res-1,1),PathStore{robot_coli(j),1}(res-1,2)-1)==1 % 在货架处躲避
%                                 
%                                 PATH= PathStore{robot_coli(j),1}(res-1:size(PathStore{robot_coli(j),1},1),:);
%                                 Path_num_MAJ=Path_num{robot_coli(j),1}(res-1:size(Path_num{robot_coli(j),1},2));
%                                 
%                                 PathStore{robot_coli(j),1}(res-1,2)=PathStore{robot_coli(j),1}(res-1,2)-1;
%                                 Path_num{robot_coli(j),1}(res-1)=PathStore{robot_coli(j),1}(res-1,2)+(PathStore{robot_coli(j),1}(res-1,1)-1)*SD;
%                                 
%                                 PathStore{robot_coli(j),1}(res:size(PathStore{robot_coli(j),1},1),:)=[];
%                                 Path_num{robot_coli(j),1}(res:size(Path_num{robot_coli(j),1},2))=[];
% 
%                                 PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1} ; PATH];
%                                 Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ];
%                                 continue
% 
%                             elseif PathStore{robot_coli(j),1}(res-1,1)-1~=0 && D(PathStore{robot_coli(j),1}(res-1,1)-1,PathStore{robot_coli(j),1}(res-1,2))==1
% 
%                                 PathStore{robot_coli(j),1}(res-1,1)=PathStore{robot_coli(j),1}(res-1,1)-1;
%                                 Path_num{robot_coli(j),1}(res-1)=PathStore{robot_coli(j),1}(res-1,2)+(PathStore{robot_coli(j),1}(res-1,1)-1)*SD;
%                                 PathStore{robot_coli(j),1}(res:size(PathStore{robot_coli(j),1},1),:)=[];
%                                 Path_num{robot_coli(j),1}(res:size(Path_num{robot_coli(j),1},2))=[];
% 
%                                 PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1} ; PATH];
%                                 Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ];
%                                 continue
%                             end
%                         end
%                             
%                         
%                         if D(PathStore{robot_coli(j),1}(res-1,1),PathStore{robot_coli(j),1}(res-1,2)+1)==1
%                             
%                             PathStore{robot_coli(j),1}(res-1,2)=PathStore{robot_coli(j),1}(res-1,2)-1;
%                             Path_num{robot_coli(j),1}(res-1)=PathStore{robot_coli(j),1}(res-1,2)+(PathStore{robot_coli(j),1}(res-1,1)-1)*SD;
%                             PathStore{robot_coli(j),1}(res:size(PathStore{robot_coli(j),1},1),:)=[];
%                             Path_num{robot_coli(j),1}(res:size(Path_num{robot_coli(j),1},2))=[];
% 
%                             PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1} ; PATH];
%                             Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ];
%                             continue
%                             
%                         elseif D(PathStore{robot_coli(j),1}(res-1,1)+1,PathStore{robot_coli(j),1}(res-1,2))==1
%                             
%                             PathStore{robot_coli(j),1}(res-1,1)=PathStore{robot_coli(j),1}(res-1,1)-1;
%                             Path_num{robot_coli(j),1}(res-1)=PathStore{robot_coli(j),1}(res-1,2)+(PathStore{robot_coli(j),1}(res-1,1)-1)*SD;
%                             PathStore{robot_coli(j),1}(res:size(PathStore{robot_coli(j),1},1),:)=[];
%                             Path_num{robot_coli(j),1}(res:size(Path_num{robot_coli(j),1},2))=[];
% 
%                             PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1} ; PATH];
%                             Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ];
%                             continue 
%                         else 
%                             disp('机器人被包围，机器人停止等待处理')
%                             PATH= PathStore{robot_coli(j),1}(res-1:size(PathStore{robot_coli(j),1},1),:);
%                             Path_num_MAJ=Path_num{robot_coli(j),1}(res-1:size(Path_num{robot_coli(j),1},2));
% 
%                             PathStore{robot_coli(j),1}(res:size(PathStore{robot_coli(j),1},1),:)=[];
%                             Path_num{robot_coli(j),1}(res:size(Path_num{robot_coli(j),1},2))=[];
% 
%                             PathStore{robot_coli(j),1}=[PathStore{robot_coli(j),1} ; PATH];
%                             Path_num{robot_coli(j),1}=[Path_num{robot_coli(j),1} Path_num_MAJ];
%                             continue
%                         end
%                     end

                    
%                     disp(res)
%                     disp(PATH)
%                     disp(Path_num_MAJ)
%                     disp(PathStore{robot_coli(j),1})
%                     disp(Path_num{robot_coli(j),1})
%                     temp(X_start,Y_start)=0;
                end
%                 
%                 if temp(X_fin,Y_fin)==1
%                      disp('终点被占用，临时释放终点');
%                      temp(X_fin,Y_fin)=0;
%                 end
                
                 if X_start > encarde && Y_start > encarde && X_start+encarde <= SD && Y_start+encarde <= SD
                             disp('横纵坐标满足大于边框的要求')
                             % 优化碰撞处理
                             if abs(Y_fin-Y_start) > encarde && abs(X_fin-X_start)> encarde
                                disp('终点在框外')
                                [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),~]=op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start(robot_coli(j)),Goal(robot_coli(j)),Path_num{robot_coli(j),1},PathStore{robot_coli(j),1},robot_coli(j),res,SD,encarde);
                              elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                disp('终点在框内')
                               % [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp] = op_modify_sup(temp,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),res,robot_coli(j),SD,D);
                                [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},~] = ori_path_op(D,temp,X_start,Y_start,X_fin,Y_fin,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},SD,robot_coli(j),encarde,res);
%                                 if RE == 1
%                               [PathStore{j,1},Path_num{j,1},temp] = ori_path_op(D,temp,X_start,Y_start,X_fin,Y_fin,PathStore{j,1},Path_num{j,1},SD,j,encarde,res);
%                                     disp('启发式算法失效')
%                                     [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp] = op_modify_sup(temp,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),res,robot_coli(j),SD,D);
%                                 end
                             else
                                 disp('终点在框外')
                                 [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),~]=op_modify_path(D,temp,X_start,Y_start,X_fin,Y_fin,Start(robot_coli(j)),Goal(robot_coli(j)),Path_num{robot_coli(j),1},PathStore{robot_coli(j),1},robot_coli(j),res,SD,encarde);
                             end
                            
                 else
                      
                             one_add_y=ones(SD,encarde);
                             one_add_x=ones(encarde,SD+2*encarde);
                             
%                              if size(temp,2) >SD
%                                  temp(:,size(temp,2))=[];
%                              end
%                              
%                              temp_am=temp;
%                              
%                              if size(temp_am,2) >SD
%                                  temp_am(:,size(temp_am,2))=[];
%                              end
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
                                [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(j),Goal(robot_coli(j))]=op_modify_path_am(temp,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(robot_coli(j)),Goal(robot_coli(j)),Path_num{robot_coli(j),1},PathStore{robot_coli(j),1},robot_coli(j),res,SD+2*encarde,SD,encarde);
                             elseif abs(Y_fin-Y_start) <= encarde && abs(X_fin-X_start)<= encarde
                                 disp('终点在框内') %% 可以尝试启发式方法，将求解范围缩小，修改temp，借鉴原始路径的启发式方法
                                 [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1}] = ori_path_op_am(D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},SD,robot_coli(j),encarde,res);
%                                if RE == 1
%                                   disp('启发式算法失效')
%                                  [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),temp] = op_modify_sup(temp,PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j)),res,robot_coli(j),SD,D);
%                                end
                             else
                                 disp('终点在框外') 
                                  [PathStore{robot_coli(j),1},Path_num{robot_coli(j),1},Start(robot_coli(j)),Goal(robot_coli(j))]=op_modify_path_am(temp,D_am_op,temp_am_op,X_start,Y_start,X_fin,Y_fin,Start(robot_coli(j)),Goal(robot_coli(j)),Path_num{robot_coli(j),1},PathStore{robot_coli(j),1},robot_coli(j),res,SD+2*encarde,SD,encarde);
                             end
                 end       
                
            end
        end
        
        %% 节点释放
        % 判断是否求解结束
%         juge=unique(Goal-Start);
%         juge_n=size(juge,2);
%         
%         MAX_ro=0;
%         
%         for i = 1:RobotNum
%                 inte=Goal(i)-Start(i);
%                 if inte == 0
%                     nu=find(Path_num{i,1}==Goal(i));
%                     MAX_ro=max([nu(1),MAX_ro]);
%                 end
%         end
%             
%         if juge_n==2
%             
%             inte=Goal-Start;
%             for i = 1:RobotNum
%                 if inte(i) ~= 0
%                     disp(MAX_ro)
%                     sup=unique(Path_num{i,1}(MAX_ro:size(Path_num{i,1},2)));
%                %     disp(sup)
%                     Path_num{i,1}(MAX_ro+1:size(Path_num{i,1},2))=[];
%                     Path_num{i,1}=[Path_num{i,1} sup];
%                     PathStore{i,1}(size(Path_num{i,1},2)+1:size(PathStore{i,1},1),:)=[];
%                     
%                     [PathStore_i,PathStore_i_i]=spread(Path_num{i,1},SD);
%                     PathStore{i,1}(:,1)=PathStore_i';
%                     PathStore{i,1}(:,2)=PathStore_i_i';
%                     
%                     disp('开始规划剩余路径并防止在某点卡死');
% 
%                     [PATH_sup,Path_num_sup] = ori_path(D,Start(i),Goal(i),SD,i);
% 
%                     PathStore{i,1}=[PathStore{i,1};PATH_sup];
%                     Path_num{i,1}=[Path_num{i,1} Path_num_sup];
%                     flag=1;
%                     Start=Goal;
%                 end
% 
%             end
%         end
        
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
        
        if isequal(Start,Goal)
             disp('路径修改全部完成')
             flag = 1;
        else
            for i = 1:RobotNum
                temp(PathStore{i,1}(res,1),PathStore{i,1}(res,2))=0; % 释放当前节点
                if res == 2
                    temp(PathStore{i,1}(res-1,1),PathStore{i,1}(res-1,2))=0; 
                end
            end
        end
        
        
        N_F=0;
        pass=[];
        for i = 1:RobotNum
            if Path_num{i,1}(res)==Goal(i) && ~ismember(i,pass)
                N_F=N_F+1;
                pass=[pass;i];
    %            disp(pass)
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
                [PATH_sup,Path_num_sup] = ori_path(D,Start(i),Goal(i),SD,i);
         %       disp(PATH_sup)
          %      disp(Path_num_sup)
                PATH_sup(size(PATH_sup,1),:)=[];
                Path_num_sup(size(Path_num_sup,2))=[];
                PathStore{i,1}=[PathStore{i,1};flipud(PATH_sup)];
                Path_num{i,1}=[Path_num{i,1} fliplr(Path_num_sup)];
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

                        [X_ST,Y_ST]=spread(Path_num{n,1}(ROB_MAX),SD);
                        [X_FI,Y_FI]=spread(Goal(n),SD);
                        X_MI=floor((X_ST+X_FI)/2);
                        Y_MI=floor((Y_ST+Y_FI)/2);
                %        disp(X_MI)
                %        disp(Y_MI)

                        if X_MI>=1 || Y_MI>=1
                            if D(X_MI,Y_MI)==0 && (abs(X_MI-X_FI)>=3 || abs(Y_MI-Y_FI)>=3) && (abs(X_MI-X_ST)>=3 || abs(Y_MI-Y_ST)>=3)
                                Start_MI=(Y_MI+(X_MI-1)*SD);
                                [~,path_num_1] = ori_path(D,Path_num{n,1}(ROB_MAX),Start_MI,SD,n);
                                [~,path_num_2] = ori_path(D,Start_MI,Goal(n),SD,n);
                                path_num_2(1)=[];
                                sup= [path_num_1 path_num_2];

                            elseif D(X_MI,Y_MI)==1 && (abs(X_MI-X_FI)>=3 || abs(Y_MI-Y_FI)>=3) && (abs(X_MI-X_ST)>=3 || abs(Y_MI-Y_ST)>=3)
                                Y_MI=Y_MI-1;
                                Start_MI=(Y_MI+(X_MI-1)*SD);
                                [~,path_num_1] = ori_path(D,Path_num{n,1}(ROB_MAX),Start_MI,SD,n);
                                [~,path_num_2] = ori_path(D,Start_MI,Goal(n),SD,n);
                                path_num_2(1)=[];
                                sup = [path_num_1 path_num_2];

                            else
                            [~,sup] = ori_path(D,Path_num{n,1}(ROB_MAX),Goal(n),SD,n); % 将原始路径规划分块
                            end

                        else
                            [~,sup] = ori_path(D,Path_num{n,1}(ROB_MAX),Goal(n),SD,n);  
                        end
                %        disp(sup)
                %        [~,sup] = ori_path(D,Path_num{n,1}(ROB_MAX),Goal(n),SD,n);     
                %         sup=unique(Path_num{n,1}(ROB_MAX:size(Path_num{n,1},2)));
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
%disp('检查路径是否存在跳跃')

% for i=1:RobotNum
%     for n = 1:length(Path_num{i,1})-1
%         if abs(Path_num{i,1}(n+1)-Path_num{i,1}(n)) ~=1 && abs(Path_num{i,1}(n+1)-Path_num{i,1}(n)) ~=SD && abs(Path_num{i,1}(n+1)-Path_num{i,1}(n)) ~=0
%             Path_num{i,1}(n+1)=Path_num{i,1}(n);
%         end
%     end
% end
% 
% disp('检查完成')
% for n =1:RobotNum
%     [PathStore{n,1}(:,1),PathStore{n,1}(:,2)]=spread(Path_num{n,1},SD);
% end

save('Path_num.mat');
save('PathStore.mat');
