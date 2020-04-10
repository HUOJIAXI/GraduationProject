function [path_rob,runtime_indi]=IP_tradion_way(D,numrobot,r_Goal,r_start)
yalmip('clear') % Yalmip接口清空内存

[obs,~]=count_obstacle(D); 
% n_obs=length(obs);          % 清点障碍物数量
size_D_m=size(D,1);
size_D_n=size(D,2);

MAXINT=2*(size_D_m+size_D_n); %大数
T=MAXINT;

[r_Goal_x,r_Goal_y]=spread(r_Goal,size_D_n);
[r_start_x,r_start_y]=spread(r_start,size_D_n);

state_rob=intvar(numrobot,2,T,'full'); %机器人在t时刻的状态，三位维决策变量，第一维表示机器人序号，第二维表示x和y，第三维表示时刻

u_rob=intvar(numrobot,2,T,'full'); %机器人在t时刻的前进状态，三维决策变量，第一维表示机器人序号，第二维表示在x方向前进还是在y方向前进

fun_obj=0;
for rob=1:numrobot
    for t =1:MAXINT
%         fun_obj=fun_obj+t*((state_rob(rob,1,t)-r_Goal_x(rob))*(state_rob(rob,1,t)-r_Goal_x(rob))+(state_rob(rob,2,t)-r_Goal_y(rob))*(state_rob(rob,2,t)-r_Goal_y(rob)));
        fun_obj=fun_obj+t*(abs(state_rob(rob,1,t)-r_Goal_x(rob))+abs(state_rob(rob,2,t)-r_Goal_y(rob)));
    end
end

C=[]; %约束集合

% 机器人状态

% 状态转移
%% 约束1 保证起点位置
h = waitbar(0,'保证起点位置');
for rob =1:numrobot
    C=[C,squeeze(state_rob(rob,:,1))-[r_start_x(rob),r_start_y(rob)]==0];
    
    per = rob / numrobot;
    waitbar(per, h ,sprintf('请等待路径连续性建立 %2.0f%%',per*100))
    
end

close(h)

%% 约束2 保证终点位置 （已在目标函数中约束）

%% 约束3 机器人需按照转移函数前进
h = waitbar(0,'机器人需按照转移函数前进');
for t = 1:T-1
    for rob=1:numrobot
        C=[C,state_rob(rob,:,t+1)==state_rob(rob,:,t)+u_rob(rob,:,t)];
%         C= [C, state_rob(rob,:,t+1)'-(A*state_rob(rob,:,t)'+B*u_rob(rob,:,t)')==0];
    end
   per = t/(T-1);
   waitbar(per, h ,sprintf('机器人需按照转移函数前进 %2.0f%%',per*100))
end
close(h)

%% 约束4 对于每次转移，x和y只能选取一个前进一个单位
% C=[C, sum(u_rob,2)<=ones(numrobot,1,MAXINT)]; % 循环向量化，对行求和
h = waitbar(0,'对于每次转移，x和y只能选取一个前进一个单位');
for rob=1:numrobot
    for t = 1:MAXINT
       C=[C, abs(u_rob(rob,1,t))+abs(u_rob(rob,2,t)) <=1];
       C=[C,u_rob(rob,1,t)>=-1];
       C=[C,u_rob(rob,1,t)<=1];
    end
    
   per = rob / numrobot;
   waitbar(per, h ,sprintf('对于每次转移，x和y只能选取一个前进一个单位 %2.0f%%',per*100))
end
close(h)
% 
% %% 约束4 机器人的起点终点需在规定区域
% %起点
% C=[C,squeeze(state_rob(:,:,1))-state_start==0];
% %终点
% for rob=1:numrobot
%     C=[C,state_rob(rob,:,T)-state_goal(rob,:)==0];
% end

%% 约束5 机器人不可与障碍物碰撞
h = waitbar(0,'机器人不可与障碍物碰撞');
for t = 1:T
    for rob=1:numrobot
        rob_s=state_rob(rob,2,t)+(state_rob(rob,1,t)-1)*size_D_n;
        for count_obs=1:length(obs)
            C=[C,(obs(count_obs)-rob_s)~=0];
        end
    end
    per = t / T;
   waitbar(per, h ,sprintf('机器人不可与障碍物碰撞%2.0f%%',per*100))
   
end     %有多重循环 可能造成内存分配时间长
close(h)

%% 约束6 机器人之间没有直接冲突
h = waitbar(0,'机器人之间没有直接冲突');
for t =1:T
    for rob=1:numrobot-1
        for rob_col=rob+1:numrobot

    %             C=[C,abs((state_rob(rob,1,t)-state_rob(rob_col,1,t)))+abs((state_rob(rob,2,t)-state_rob(rob_col,2,t)))~=0];
               state_rob_1=state_rob(rob,:,t);
               state_rob_col_1=state_rob(rob_col,:,t);
               C=[C,(state_rob_1-state_rob_col_1)~=0];
        end
    end
   per = t / T;
   waitbar(per, h ,sprintf('机器人之间没有直接冲突 %2.0f%%',per*100))
end
close(h)

%% 约束6 机器人之间没有交叉冲突
h = waitbar(0,'机器人之间没有交叉冲突');
for t =1:T-1
    for rob=1:numrobot-1
        for rob_col=rob+1:numrobot
            
    %             C=[C,abs(state_rob(rob,1,t+1)+state_rob(rob,1,t)-(state_rob(rob_col,1,t+1)+state_rob(rob_col,1,t)))+abs(state_rob(rob,2,t+1)+state_rob(rob,2,t)-(state_rob(rob_col,2,t+1)+state_rob(rob_col,2,t)))~=0];
                state_rob_1=state_rob(rob,:,t+1);
                state_rob_2=state_rob(rob,:,t);
                state_rob_col_1=state_rob(rob_col,:,t+1);
                state_rob_col_2=state_rob(rob_col,:,t);
    %             
                C=[C,(state_rob_1+state_rob_2-(state_rob_col_1+state_rob_col_2))~=0];
    %             C=[C,(state_rob(rob,:,t)-state_rob(rob_col,:,t))~=[1,1]];
    %               C=[C,(state_rob(rob,:,t+1)+state_rob(rob,:,t)-(state_rob(rob_col,:,t+1)+state_rob(rob_col,:,t)))~=0];

        end
    end
       per = t / (T-1);
       waitbar(per, h ,sprintf('机器人之间没有交叉冲突 %2.0f%%',per*100))
end
    
close(h)


%% 约束7 机器人位置必须在范围内
C=[C,state_rob(:,1,:)>0,state_rob(:,2,:)>0,state_rob(:,1,:)<=size_D_m,state_rob(:,2,:)<=size_D_n];

ops = sdpsettings('verbose',1,'solver','gurobi');
% 求解
result  = optimize(C,fun_obj,ops);

save dat result
runtime_indi=result.solvertime;
if result.problem== 0
%    value(z)
%     disp(value(dir_rob))
%    text=' 系统总最优路径长度：';
%    disp([text,num2str(value(z))]);
    disp('系统总最优路径长度：Best objective');
else
    disp('Finish ! ');
%     disp(value(dir_rob))
end

path_rob=cell(numrobot,1);

path_total=value(state_rob);

for rob = 1:numrobot
    path_rob{rob,1}=squeeze(path_total(rob,:,:))';
end




