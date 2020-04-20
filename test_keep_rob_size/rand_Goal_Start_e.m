function [r_Goal,r_start]=rand_Goal_Start_e(D,numrobot)
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
r_start = randperm(length(obs),numrobot);
r_Goal = randperm(length(obs),numrobot);

for i = 1:numrobot
    r_start(i)=obs(r_start(i));
    r_Goal(i)=obs(r_Goal(i));
end



flag=1;
num_flag_error=0;
num_flag_total=0;


while flag==1
%    test=[r_start r_Goal];
%     len_start_ori =length(r_start);
%     len_Goal_ori  =length(r_start);
%     len_start     =length(unique(r_start));
%     len_Goal      =length(unique(r_Goal));
    test=r_start-r_Goal;
    num_flag_total=num_flag_total+1;
    
%     len_ori=length(test);
%     len=length(unique(test));
    if  ismember(0,test)
%        len_ori~=len || || || len_start_ori~=len_start || len_Goal_ori ~=len_Goal
        flag = 1;
        num_flag_error=num_flag_error+1;
        r_start = randperm(length(obs),numrobot);
        r_Goal = randperm(length(obs),numrobot);

        for i = 1:numrobot
            r_start(i)=obs(r_start(i));
            r_Goal(i)=obs(r_Goal(i));
        end
        continue
    end
    
    flag = 0;
    
    
end

[start_x,start_y]=spread(r_start,n);
[goal_x,goal_y]=spread(r_Goal,n);

for i = 1: numrobot
    start_x(i)=start_x(i)-1;
    r_start(i)=start_y(i)+(start_x(i)-1)*n;
    goal_x(i)=goal_x(i)-1;
    r_Goal(i)=goal_y(i)+(goal_x(i)-1)*n;
    
%     if mod(start_y(i)/2,2)==1
%         start_y(i)=start_y(i)-1;
%         r_start_ori(i)=start_y(i)+(start_x(i)-1)*n;
%     elseif mod(start_y(i)/2,2)==0
%         start_y(i)=start_y(i)+1;
%         r_start_ori(i)=start_y(i)+(start_x(i)-1)*n;
%     elseif mod(start_y(i),2)==1
%         start_x(i)=start_x(i)+1;
%         r_start_ori(i)=start_y(i)+(start_x(i)-1)*n;
%     end
%     
%     if mod(goal_y(i)/2,2)==1
%         goal_y(i)=goal_y(i)-1;
%         r_Goal_ori(i)=goal_y(i)+(goal_x(i)-1)*n;
%     elseif mod(goal_y(i)/2,2)==0
%         goal_y(i)=goal_y(i)+1;
%         r_Goal_ori(i)=goal_y(i)+(goal_x(i)-1)*n;
%     elseif mod(goal_y(i),2)==1
%         goal_x(i)=goal_x(i)+1;
%         r_Goal_ori(i)=goal_y(i)+(goal_x(i)-1)*n; % 最后进行执行
%     end
    
end

% 
% disp(num_flag_total);
% disp(num_flag_error);