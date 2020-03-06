function [start_re,goal_re,start_sp,goal_sp,D_reduit] = reduit(Start,Goal,D)

sizeD=size(D,2);
[start_sp_X,start_sp_Y]=spread(Start,sizeD);
[Goal_sp_X,Goal_sp_Y]=spread(Goal,sizeD);

for i =1:length(start_sp_Y)
    if mod(start_sp_Y(i),2)==0
        
        if mod(start_sp_Y(i),2)==1
                start_sp_Y(i)=start_sp_Y(i)+1;
        end
        if mod(start_sp_Y(i),2)==0
                start_sp_Y(i)=start_sp_Y(i)-1;
        end
        
    end
end

start_sp=start_sp_Y+(start_sp_X-1)*sizeD;

for i =1:length(Goal_sp_Y)
    if mod(Goal_sp_Y(i),2)==0
        if mod(Goal_sp_Y(i),2)==1
            Goal_sp_Y(i)=Goal_sp_Y(i)+1;
        end
        if mod(Goal_sp_Y(i),2)==0
            Goal_sp_Y(i)=Goal_sp_Y(i)-1;
        end
    end
end

goal_sp=Goal_sp_Y+(Goal_sp_X-1)*sizeD;
D_reduit=[];

for i =1:sizeD
    if mod(i,2)~=0
       D_reduit = [D_reduit D(:,i)];
    end
end

% start_sp_ori=unique(start_sp);
% goal_sp_ori=unique(goal_sp);

for i =1:length(start_sp_Y)
    if start_sp_Y(i)==1
        start_sp_Y(i)=1;
    else
        start_sp_Y(i) = start_sp_Y(i)-(start_sp_Y(i)-1)/2;
    end
end

for i =1:length(Goal_sp_Y)
    if Goal_sp_Y(i)==1
        Goal_sp_Y(i)=1;
    else
        Goal_sp_Y(i) = Goal_sp_Y(i)-(Goal_sp_Y(i)-1)/2;
    end
end

sizeDre =size(D_reduit,2);

start_re=start_sp_Y+(start_sp_X-1)*sizeDre;

goal_re =Goal_sp_Y+(Goal_sp_X-1)*sizeDre;

% start_re_ori=unique(start_re,'stable');
% goal_re_ori=unique(goal_re,'stable');
%num=length(start_re_ori);


