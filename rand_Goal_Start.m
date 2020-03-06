function [r_Goal,r_start]=rand_Goal_Start(D,numrobot)
m=size(D,1);
n=size(D,2);
r_start = round(1 + (m*n-1).*rand([1 numrobot]));
r_Goal = round(1 + (m*n-1).*rand([1 numrobot]));
obs=[];
for i = 1:m
    for j = 1:n
        if D(i,j) == 1
            obs=[obs j+(i-1)*n];
        end
    end
end

flag=1;

while flag==1
    test=[r_start r_Goal];
    len_start_ori =length(r_start);
    len_Goal_ori  =length(r_Goal);
    len_start     =length(unique(r_start));
    len_Goal      =length(unique(r_Goal));
    
    len_ori=length(test);
    len=length(unique(test));
    if len_ori~=len || ~isempty(intersect(r_start,obs)) || ~isempty(intersect(r_Goal,obs)) || len_start_ori~=len_start || len_Goal_ori ~=len_Goal
        flag = 1;
        r_start = round(1 + (m*n-1).*rand([1 numrobot]));
        r_Goal = round(1 + (m*n-1).*rand([1 numrobot]));
    else
        flag=0;
    end
end