function [start_re,goal_re,D_reduit] = reduit(Start,Goal,D)

sizeD=size(D,2);
[start_sp_X,start_sp_Y]=spread(Start,sizeD);
[Goal_sp_X,Goal_sp_Y]=spread(Goal,sizeD);

for i =1:length(start_sp_X)
    if i >1
        if mod(start_sp_X(i)+1,3)==0
%                 start_sp_X(i)=start_sp_X(i)+1;
        end
        if mod(start_sp_X(i),3)==0
                start_sp_X(i)=start_sp_X(i)-1;
        end        
    end
end


for i =1:length(Goal_sp_X)
    if i >1
        if mod(Goal_sp_X(i)+1,3)==0
%             Goal_sp_X(i)=Goal_sp_X(i)-1;
        end
        if mod(Goal_sp_X(i),3)==0
            Goal_sp_X(i)=Goal_sp_X(i)-1;
        end
    end
end
D_reduit=[];

for i =1:sizeD
    if mod(i,3)~=0 || i==1
       D_reduit=[D_reduit;D(i,:)];
    end
end


for i =1:length(start_sp_X)
    if start_sp_X(i)<=2
        start_sp_X(i)=start_sp_X(i);
    else
        start_sp_X(i) = start_sp_X(i)-fix(start_sp_X(i)/3);
    end
end

for i =1:length(Goal_sp_Y)
    if Goal_sp_X(i)<=2
        Goal_sp_X(i)=Goal_sp_X(i);
    else
        Goal_sp_X(i) = Goal_sp_X(i)-fix(Goal_sp_X(i)/3);
    end
end


sizeDre =size(D_reduit,2);

start_re=start_sp_Y+(start_sp_X-1)*sizeDre;

goal_re =Goal_sp_Y+(Goal_sp_X-1)*sizeDre;

% size(D_reduit)
for i=1:length(start_sp_Y)
    if D_reduit(Goal_sp_X(i),Goal_sp_Y(i)) ==1 || D_reduit(start_sp_X(i),start_sp_Y(i)) ==1
%         disp(i)
        disp('起点终点选择错误')
        return
    end
end

disp('起点终点审查通过')

% start_re_ori=unique(start_re,'stable');
% goal_re_ori=unique(goal_re,'stable');
%num=length(start_re_ori);


