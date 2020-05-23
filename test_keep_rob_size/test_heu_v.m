flag_test=1;
while 1
%     clear;
    clc;
        size_D_index=19;
        [D]=construct_D(size_D_index);

        m=size(D,1);
        n=size(D,2);
        % a=floor(m/2);
        % b=floor(n/2);
        %rng shuffle

        nobs=[];
        obs=[];
        for i = 1:m
            for j = 1:n
                if D(i,j) == 1
                    obs=[obs j+(i-1)*n];
        %             [obs_x,obs_y]=spread(obs,m);
                else
                    nobs=[nobs j+(i-1)*n];
                end
            end
        end

        count_nobs=length(nobs);
       
        size_D=size(D,1);
        
    [Goal_ori,Start_ori]=rand_Goal_Start_e(D,count_nobs);
    %RobotNum = size(Start,2);

%     [Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);
    % disp(D_reduit)
%     size_D=size(D_reduit,2);
    disp(datestr(now));


    test_choix=randperm(count_nobs,RobotNum_total);

    Start_test=Start_ori(test_choix);
    %                 disp(Start_test)
    Goal_test=Goal_ori(test_choix);

    [err_heu,ini_Path_num,ini_PathStore]=initial_x_way(D,RobotNum_total,Start_test,Goal_test);
    
    flag_test=flag_test+1;
end