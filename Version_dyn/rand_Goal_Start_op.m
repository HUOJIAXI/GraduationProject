function [r_Goal,r_start,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,numrobot)
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
r_start_ori = zeros(1,numrobot);
r_Goal_ori = zeros(1,numrobot);

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
    if mod(start_x(i)+1,3)==0
        start_x(i)=start_x(i)-1;
        r_start_ori(i)=start_y(i)+(start_x(i)-1)*n;
        continue
    elseif mod(start_x(i),3)==0
        start_y(i)=start_y(i)-1;
        r_start_ori(i)=start_y(i)+(start_x(i)-1)*n;
        continue
    end
    
end

for i = 1:numrobot
    if mod(goal_x(i)+1,3)==0
        r_Goal_ori(i)=goal_y(i)+(goal_x(i)-1-1)*n;
        continue
    elseif mod(goal_x(i),3)==0
        r_Goal_ori(i)=goal_y(i)+1+(goal_x(i)-1)*n;
        continue
    end
end

