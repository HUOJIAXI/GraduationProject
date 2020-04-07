function [Start,Goal,chose]=Test_disturb(Start,Goal,D)

chose=randperm(length(Start),1);

m=size(D,1);
n=size(D,2);
% a=floor(m/2);
% b=floor(n/2);
%rng shuffle

nobs=[];
obs=[];
numrobot=1;

posibilite=randperm(2,1);

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

r_start = randperm(length(obs),numrobot);
r_Goal = randperm(length(obs),numrobot);

if posibilite==1
    disp('终点发生扰动')
    r_start=Start(chose);
    r_Goal=obs(r_Goal);
    [goal_x,goal_y]=spread_sin(r_Goal,n);
    goal_x=goal_x+1;
    r_Goal=goal_y+(goal_x-1)*n;
    
    flag=1;
    num_flag_error=0;

    while flag==1
        test=r_start-r_Goal;

        if  test==0
            flag = 1;
            num_flag_error=num_flag_error+1;
            r_Goal = randperm(length(obs),numrobot);
            r_Goal=obs(r_Goal);
            [goal_x,goal_y]=spread_sin(r_Goal,n);
            goal_x=goal_x+1;
            r_Goal=goal_y+(goal_x-1)*n;
            
            continue
        end

        flag = 0;
    end
    Goal(chose)=r_Goal;
end

if posibilite==2
    disp('起点发生扰动')
    r_Goal=Goal(chose);
    r_start=obs(r_start);
    [start_x,start_y]=spread_sin(r_start,n);
    start_x=start_x+1;
    r_start=start_y+(start_x-1)*n;
    
    flag=1;
    num_flag_error=0;

    while flag==1
        test=r_start-r_Goal;

        if  test==0
            flag = 1;
            num_flag_error=num_flag_error+1;
            r_start = randperm(length(obs),numrobot);
            r_Goal=Goal(chose);
            r_start=obs(r_start);
            [start_x,start_y]=spread_sin(r_start,n);
            start_x=start_x+1;
            r_start=start_y+(start_x-1)*n;
            
            continue
        end

        flag = 0;


    end
    Start(chose)=r_start;
end
