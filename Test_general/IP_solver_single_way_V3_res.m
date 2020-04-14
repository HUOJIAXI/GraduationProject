
%% Version: 14/03/2020
% Author:HUO JIAXI
% 引入单行道限制
%% 待修改
function [err,PATH,Path,value_dir_way,runtime_indi]=IP_solver_single_way_V3_res(D,l,t,numrobot,size_D,ini_x_value)
yalmip('clear')
PATH=cell(numrobot,1);
Path=cell(numrobot,1);
o_single=cell(numrobot,1);
flag_cross=0; % 是否考虑交汇点约束 1：考虑 0：不考虑

if numrobot <=16
    timelimit=round(5*numrobot);
else
    timelimit=round(100*numrobot);
end

m_D=size(D,1);
n_D=size(D,2);
num_way=((m_D+1)*(n_D-1)+(m_D-1)*(n_D+1))/4; % 需要利用节点方向推出边占用情况

d = transfer(D);
m = size(d,1); 
n = size(d,2); 
% 决策变量
nobs=[];
obs=[];
for i = 1:m_D
    for j = 1:n_D
        if D(i,j) == 1
            obs=[obs j+(i-1)*n_D]; % 障碍物所在位置
%             [obs_x,obs_y]=spread(obs,m);
        else
            nobs=[nobs j+(i-1)*n_D];
        end
    end
end

x = binvar(n,n,numrobot,'full'); % n*n维的决策变量
% u = intvar(numrobot,n,'full');
% dir = intvar(numrobot,n,n,'full'); % a 表示每个机器人的方向
if flag_cross==1
    dir_way=intvar(1,num_way,'full');
end
dir_rob=intvar(numrobot,num_way,'full');


%p = intvar((n-1)/2,1);

% 目标

z=0;
for i = 1:numrobot
    z = z+sum(sum(d.*squeeze(x(:,:,i))));
end
% 
% for i = 1:numrobot
%     z = z+sum(min(dir_rob(i,:),1));
% end

%z = sum(sum(d.*x(1,:,:)));

% 约束添加
C = [];
%%后期作为参数在调用中赋值
%%
% 静止不动时返回0
same=[];
temp_l=[];
temp_t=[];
temp_ini=[];
flag_same=0;
for i = 1:numrobot
    if l(i) == t(i)
        [N,B]=spread_sin(t,n);
        PATH{i,1} = [N,B];
        Path{i,1}(1) = l(i);
        disp('二次检查：起点终点在同一个点')
        same=[same i];
        flag_same=1;
        value_dir_way=0;
        runtime_indi=0;
        err=1;
        return
    end
end

if flag_same==1
    for i =1:numrobot
        if ~ismember(i,same)
            temp_l=[temp_l l(i)];
            temp_t=[temp_t t(i)];
            
            temp_ini=cat(3,temp_ini,ini_x_value(:,:,i));
        end
    end
    
    l=temp_l;
    t=temp_t;
    ini_x_value=temp_ini;
    numrobot=numrobot-length(same); 
else
    disp('二次检查：不存在起点终点在同一个点的情况')
    err=0;
end

disp('正在进行约束建立')
%% 约束1 确保路径从起点出发并在终点结束
% tic
h = waitbar(0,'请等待路径连续性建立');
for i = 1:numrobot
    C = [C, sum(x(l(i),:,i)) - x(l(i),l(i),i) - sum(x(:,t(i),i)) + x(t(i),t(i),i)== 0, sum(x(l(i),:,i)) - x(l(i),l(i),i) == 1, sum(x(:,l(i),i)) - x(l(i),l(i),i) - sum(x(t(i),:,i)) + x(t(i),t(i),i) == 0,sum(x(:,l(i),i)) - x(l(i),l(i),i) == 0];
    per = i / numrobot;
    waitbar(per, h ,sprintf('请等待路径连续性建立 %2.0f%%',per*100))
end
close(h)
% toc
disp('约束1 确保路径从起点出发并在终点结束 建立完成')
%% 约束2 确保出入边条件，每个顶点在路径中仅出现一次 约束3 避免出现子循环
% tic
h = waitbar(0,'请等待顶点限制建立');
for i = 1:numrobot
%               向量化前代码：由于是全图约束 k为机器人，i为1:n，子循环的出现不满足最优，会被去除
%             C = [C, sum(x(i,:,k))-x(i,i,k)- sum(x(:,i,k))+x(i,i,k) == 0];
%             C = [C, sum(x(i,:,k))-x(i,i,k) <= 1];
%             C = [C, sum(x(:,i,k))-x(i,i,k) <= 1];

    dead=sort(unique([l(i),t(i)]));

    m1=squeeze(sum(x(:,:,i),1))';
    m2=squeeze(sum(x(:,:,i),2));
    dia=[];

    temp=diag(x(:,:,i));
    dia=cat(2,dia,temp(:));

    m1(dead,:)=[];
    m2(dead,:)=[];
    dia(dead,:)=[];

    c1= (m2-m1)==zeros(size(m1,1),size(m1,2));
    c2= (m2-dia) <=ones(size(m1,1),size(m1,2));
    c3= (m1-dia) <=ones(size(m1,1),size(m1,2));

    C = [C,c1,c2,c3];
    per = i / numrobot;
    waitbar(per, h ,sprintf('请等待顶点限制建立 %2.0f%%',per*100))
end

close(h)
% toc
disp('约束2 确保出入边条件，每个顶点在路径中仅出现一次 建立完成')

% u = intvar(numrobot,n,'full');
% for k=1:numrobot
%     for i = 1:n
%         for j = 1:n
%             if i~=j && i ~=l(k) && i ~=t(k) && j ~=l(k) && j ~=t(k)
%                 C = [C,u(k,i)-u(k,j) + (n-3)*x(i,j,k)<=n-4];
%             end
%         end
%     end
% end

%% 约束4 单行线法则（交叉点不可只出不进或只进不出）
% tic
h = waitbar(0,'请等待巷道方向建立');

for k = 1:numrobot
for i = 1:m
    [i_x,i_y]=spread_sin(i,size_D);
%     [j_x,j_y]=spread_sin(j,size_D);
    if mod(i,2)==0  && D(i_x,i_y) ~=1 
        if mod(i_x,2)==1 && mod(i_y,2)==0
            if i ==l(k)
                dir_rob(k,i/2) = x(l(k),i+1,k)+3*x(l(k),i-1,k);
            elseif i ==t(k)
                dir_rob(k,i/2) = x(i-1,t(k),k)+3*x(i+1,t(k),k);
%             if i+1~=t
            else
                dir_rob(k,i/2) = x(i,i+1,k)+3*x(i+1,i,k);
            end
%             elseif i+1 ==t
%                 dir_rob(k,i/2) = x(k,i-1,i)+3*x(k,i,i-1);
%             end
        end
        if mod(i_x,2)==0 && mod(i_y,2)==1
%             if i+size_D~=t
           if i == l(k)
               dir_rob(k,i/2) = x(l(k),i+size_D,k)+3*x(l(k),i-size_D,k);
           elseif i ==t(k)
               dir_rob(k,i/2) = x(i-size_D,t(k),k)+3*x(i+size_D,t(k),k);
           else
                dir_rob(k,i/2) = x(i,i+size_D,k)+3*x(i+size_D,i,k);
           end
                
%             elseif i+size_D==t
%                 dir_rob(k,i/2) = x(k,i-size_D,i)+3*x(k,i,i-size_D);
%             end
        end
    end
end
    per = k / numrobot;
    waitbar(per, h ,sprintf('请等待巷道方向建立 %2.0f%%',per*100))
end
close(h)
% toc
% 
disp('约束3 单行线法则 建立完成')
%% 约束5 单行线法则 （巷道方向框定）
% tic
h = waitbar(0,'请等待巷道方向确认');
a=max(dir_rob,[],1);
% disp(size(a))
% %     [i,j]=spread_sin(k,size_D);
for k =1:num_way
    for rob = 1:numrobot
             C = [ C, a(k)-dir_rob(rob,k) ~=2 ];
    end  
    per = k / num_way;
    waitbar(per, h ,sprintf('请等待巷道方向确认%2.0f%%',per*100))
end
close(h)
% %      
% a=max(dir_rob,[],1)';
% disp(size(a));
% 
% t=ones(1,size(dir_rob,1));
% disp(size(t))
% b=a(:,t)';
% disp(size(b))
% disp(size(dir_rob))
% C=[C,(b-dir_rob)~=2];

% rob_ran=[(1:numrobot),(1:num_way)];
% C=[C, 0<=dir_rob(rob_ran)<=3]; 
for k =1:num_way
     C=[C, 0<=dir_rob(1:numrobot,k)<=3];
end
% toc
disp('约束4 巷道方向确认 建立完成')


%% 交汇点限制
% flag_cross=0;
if flag_cross==1
    
    for k = 1:num_way
     dir_way(k) = max(dir_rob(:,k));
    end


    for k = 1:num_way
     C=[C, 0<=dir_way(k)<=3];
    end
     
    for k = 1:n 

        [i,j]=spread_sin(k,size_D);

        %% 周围四个
         if (i==1&&j==1)
            C = [C,  dir_way((j+(i-1)*n_D+1)/2)+dir_way((j+(i)*n_D)/2)~=2];
            C = [C,  dir_way((j+(i-1)*n_D+1)/2)+dir_way((j+(i)*n_D)/2)~=6];
    %         C = [C,  dir_way((j+(i-1)*size_D+1)/2)+dir_way((j+(i)*size_D)/2)~=6];
         end

         if (i==m_D&&j==n_D)
            C = [C,  dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1-1)*n_D)/2)~=2];
            C = [C,  dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1-1)*n_D)/2)~=6];
    %         C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)~=6];
         end

         if (i==1&&j==n_D)
            C = [C,  dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1+1)*n_D)/2)~=4];
    %         C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)~=2];
    %         C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)~=6];
         end

         if (i==m_D&&j==1)
            C = [C,  dir_way((j+(i-1-1)*n_D)/2)+dir_way((j+1+(i-1)*n_D)/2)~=4];
    %         C = [C,  dir_way((j+(i-1-1)*size_D)/2)+dir_way((j+1+(i-1)*size_D)/2)~=6];
         end

         %% 边缘
         if numrobot >2
         if i==1||i==m_D
            if mod(j,2)==1 && i==1 && j > 1 && j < n_D
               C= [C,  dir_way((j+1+(i-1)*n_D)/2)+ dir_way((j+(i+1-1)*n_D)/2)-dir_way((j-1+(i-1)*n_D)/2) ~=5];
    %            C= [C,  dir_way((j+1+(i-1)*size_D)/2)+ dir_way((j+(i+1-1)*size_D)/2)-dir_way((j-1+(i-1)*size_D)/2) ~=-1];
               C= [C,  (dir_way((j+1+(i-1)*n_D)/2)+ dir_way((j+(i+1-1)*n_D)/2))/2*3-max(dir_way((j-1+(i-1)*n_D)/2),1)~=0];
            end

            if mod(j,2)==1 && i==m_D && j > 1 && j < n_D
                C= [C, (dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1-1)*n_D)/2))/2*3-max(dir_way((j+1+(i-1)*n_D)/2),1) ~=0];
    %            C= [C, dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)-dir_way((j+1+(i-1)*size_D)/2)  ~=-1];
               C= [C, dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1-1)*n_D)/2)-dir_way((j+1+(i-1)*n_D)/2)  ~= 5];
            end



        elseif mod(i,2)==1 && i>1 && i<m_D
            if j==1
               C= [C, dir_way((j+1+(i-1)*n_D)/2) + dir_way((j+(i-1+1)*n_D)/2)-dir_way((j+(i-1-1)*n_D)/2) ~=5];
    %            C= [C, dir_way((j+1+(i-1)*size_D)/2) + dir_way((j+(i-1+1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2) ~=-1];
               C= [C, (dir_way((j+1+(i-1)*n_D)/2) + dir_way((j+(i-1+1)*n_D)/2))/2*3-max(dir_way((j+(i-1-1)*n_D)/2),1)~=0];
            end

            if j==size_D
               C= [C, (dir_way((j-1+(i-1)*n_D)/2) + dir_way((j+(i-1-1)*n_D)/2))/2*3-max(dir_way((j+(i-1+1)*n_D)/2),1)~=0];
    %            C= [C, dir_way((j-1+(i-1)*size_D)/2) + dir_way((j+(i-1-1)*size_D)/2)-dir_way((j+(i-1+1)*size_D)/2) ~=-1];
               C= [C, dir_way((j-1+(i-1)*n_D)/2) + dir_way((j+(i-1-1)*n_D)/2)-dir_way((j+(i-1+1)*n_D)/2) ~=5];
            end
         end

        %% 中部
         if numrobot >= 4

             if mod(i,2)==1 &&  i>1 && i < m_D && mod(j,2)==1 && j>1 && j < n_D

    %             C = [C, dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)-2*max(dir_way((j-1+(i-1)*size_D)/2),dir_way((j+(i-1+1)*size_D)/2))+dir_way((j+1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)-2*max(dir_way((j+(i-1-1)*size_D)/2),dir_way((j+1+(i-1)*size_D)/2))~= 0];

            C = [C, dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1+1)*n_D)/2)-dir_way((j+1+(i-1)*n_D)/2)-dir_way((j+(i-1-1)*n_D)/2)~= 4];
    %         C = [C, max(dir_way((j-1+(i-1)*size_D)/2),1)+max(dir_way((j+(i-1+1)*size_D)/2),1)-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= -4];
            C = [C, (dir_way((j-1+(i-1)*n_D)/2)+dir_way((j+(i-1+1)*n_D)/2))*3-(max(dir_way((j+1+(i-1)*n_D)/2),1)+max(dir_way((j+(i-1-1)*n_D)/2),1))~= 0];

    %         C = [C, (dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2))/2-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= 1];
    %         C = [C, (dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2))/2-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= -5];

            end

         end 
         end
    end
end
  
% index=[(1:n),(1:n),(1:numrobot)];
% for k=1:numrobot
%     for i = 1:n
%         for j = 1:n
%             if i==j 
%                 C = [C,x(i,j,k)==0];
%             end
%         end
%     end
% end

%% 求解IP模型

% 参数设置

% % [ini_dir_way] = initial();
% [ini_dir_way] = initial(n_D,m_D);
% assign(dir_way,ini_dir_way);
disp('启动求解器')
% assign(x,ini_x_value);
% ops = sdpsettings('verbose',1,'solver','gurobi','usex0',1,'gurobi.TimeLimit',timelimit);%verbose计算冗余量，值越大显示越详细
ops = sdpsettings('verbose',1,'solver','gurobi','usex0',0,'gurobi.TimeLimit',timelimit);
%ops = sdpsettings('verbose',0,'solver','cplex');
% 求解
result  = optimize(C,z,ops);
% x_value=value(x);

% disp(result)
save dat result
% save dat x_value
runtime_indi=result.solvertime;
if result.problem== 0
%    value(z)
%     disp(value(dir_rob))
%    text=' 系统总最优路径长度：';
%    disp([text,num2str(value(z))]);
    disp('系统总最优路径长度：Best objective');
else
    disp('系统总最优路径长度：Best objective ');
%     disp(value(dir_rob))
end

if flag_cross == 0
    for k = 1:num_way
     dir_way(k) = max(dir_rob(:,k));
    end
end
    
value_dir_way=value(dir_way);
%disp(value(dir))
o=value(x);

%% 邻接矩阵转换，路径绘制

for i = 1:numrobot
    o_single{i,1} =  squeeze(o(:,:,i)) ; 
end

for i = 1:numrobot
    o_sin=o_single{i,1};
%    disp(o_sin)
    Path{i,1}=solvermatrix(o_sin,l(i),t(i));
    m = size(D,2);
    [X,Y]=spread(Path{i,1},m);
    PATH{i,1}=cat(1,X,Y)'; % 路径存入PATH matrix
end


