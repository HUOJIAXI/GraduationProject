function [Start_test,Goal_test,dis_num,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=disturb(Start_test,Goal_test,D,RobotNum_total,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value)

D_size=size(D,1);

dis_num=1+round(rand(1,1)*(RobotNum_total-1));

dis_flag=round(rand(1,1));

% dis_flag=1;

% 
% ini_u_value(dis_num,:)=0;
% ini_x_value(:,:,dis_num)=0;
% ini_dir_rob(dis_num,:)=0;
% 
% [ini_dir_way_res] = initial(D_size);
% ini_dir_way=ini_dir_way_res;

if dis_flag==0
    flag=0;
    disp('起点受到扰动')
    temp_Start=Start_test(dis_num);
    while flag==0
        [rand_x,rand_y]=spread_sin(Start_test(dis_num),D_size);

        if mod(rand_x,2) ==1
            if rand_y<D_size
                rand_y = rand_y + 1;
            elseif rand_y == D_size
                rand_y = rand_y - 1;
            end

        end

        if mod(rand_x,2) == 0
            if rand_x<D_size
                rand_x = rand_x + 1;
            elseif rand_x == D_size
                rand_x = rand_x - 1;
            end

        end

        Start_test(dis_num) = rand_y + (rand_x-1)*D_size;
        
        if  Start_test(dis_num) == Goal_test(dis_num)
            continue
        else
            flag=1;
        end
    end
    
    if ini_x_value(Start_test(dis_num),temp_Start,dis_num)==1
        ini_x_value(Start_test(dis_num),temp_Start,dis_num)=0;
    elseif ini_x_value(temp_Start,Start_test(dis_num),dis_num)==1
        ini_x_value(temp_Start,Start_test(dis_num),dis_num)=0;
    elseif ini_x_value(Start_test(dis_num),temp_Start,dis_num)==0
        ini_x_value(Start_test(dis_num),temp_Start,dis_num)=1; 
    end
    
    
else
    flag = 0;
    disp('终点受到扰动')
    temp_Goal=Goal_test(dis_num);
    flag_full=0;
    flag_full_y=0;
    while flag ==0
        [rand_x,rand_y]=spread_sin(Goal_test(dis_num),D_size);

        if mod(rand_x,2) ==1
            if rand_y<D_size
                rand_y = rand_y + 1;
            elseif rand_y == D_size
                rand_y = rand_y - 1;
                 flag_full=1;
            end

        end

        if mod(rand_x,2) == 0
            if rand_x<D_size
                rand_x = rand_x + 1;
                 flag_full_y=1;
            elseif rand_x == D_size
                rand_x = rand_x - 1;
                 flag_full=1;

            end

        end

        Goal_test(dis_num) = rand_y + (rand_x-1)*D_size;
        
        if  Start_test(dis_num) == Goal_test(dis_num)
            continue
        else
            flag=1;
        end
    end
    
    if ini_x_value(temp_Goal,Goal_test(dis_num),dis_num)==0 && flag_full==0 && flag_full_y==0
        ini_x_value(temp_Goal,Goal_test(dis_num),dis_num)=1;
    elseif ini_x_value(temp_Goal,Goal_test(dis_num),dis_num)==1
        ini_x_value(temp_Goal,Goal_test(dis_num),dis_num)=0;
    elseif ini_x_value(Goal_test(dis_num),temp_Goal,dis_num)==1
        ini_x_value(Goal_test(dis_num),temp_Goal,dis_num)=0;

    end
    
end
