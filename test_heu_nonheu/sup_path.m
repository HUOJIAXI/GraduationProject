function [RE,PATH,path_num] =sup_path(D,D_ori,Start,Goal,SD,i)
    [X_start,Y_start] = spread_sin(Start,SD);
%     disp(Start)
%     disp(Goal)
%     disp(D_ori)
    [X_fin,Y_fin] = spread_sin(Goal,SD);
    squ=max(abs(X_start-X_fin),abs(Y_start-Y_fin));
    ini_x=min(X_start,X_fin);
    ini_y=min(Y_start,Y_fin);
    
    Start_op_x=X_start-ini_x+1;
    Start_op_y=Y_start-ini_y+1;
    Goal_op_x =X_fin  -ini_x+1;
    Goal_op_y =Y_fin  -ini_y+1;
    
    if (ini_x+squ)<=SD && (ini_y+squ)<=SD
        
        disp('开始启发式规划从备用终点到原始终点的路径')

        Start_op=Start_op_y+(Start_op_x-1)*(squ+1);
        Goal_op = Goal_op_y+(Goal_op_x-1)*(squ+1);
%          disp(D_ori)
        temp_D=D_ori(ini_x:ini_x+squ,ini_y:ini_y+squ);
%         disp(Start_op)
%         disp(Goal_op)
        temp_D(Start_op_x,Start_op_y)=0;
        temp_D(Goal_op_x,Goal_op_y)=0;

        [RE,PATH,~]=IP_solver_op(temp_D,Start_op,Goal_op,i);
        if RE ==1
        PATH=[[X_start,Y_start];[X_fin,Y_fin]];
        end
        
        PATH=PATH(2:length(PATH),:);
%         disp(PATH)
        PATH(:,1)=PATH(:,1)+ini_x-1;
        PATH(:,2)=PATH(:,2)+ini_y-1;
        %path_num=path_num+(ini_y-1)+(ini_x-1)*SD;
        path_num=(PATH(:,2)+(PATH(:,1)-1)*SD)';
    else
        disp('开始普通规划从备用终点到原始终点的路径')
        D_ori(X_fin,Y_fin)=0;
        D_ori(X_start,Y_start)=0;
%         disp(D_ori)
%         disp(Start)
%         disp(Goal)
        [RE,PATH,path_num]=IP_solver_op(D_ori,Start,Goal,i);
        if RE ==1
            PATH=[[X_start,Y_start];[X_fin,Y_fin]];
            path_num=[Start Goal];
        end
        PATH=PATH(2:length(PATH),:);
        path_num=path_num(2:length(path_num));
    end