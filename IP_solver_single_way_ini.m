
%% Version: 14/03/2020
% Author:HUO JIAXI
% 引入单行道限制
%% 待修改
function [PATH,Path,dir_way_value,dir_rob_value,x_value,u_value]=IP_solver_single_way_ini(D,l,t,numrobot,size_D,ini_dir_way)
PATH=cell(numrobot,1);
Path=cell(numrobot,1);
o_single=cell(numrobot,1);

timelimit=round(100*numrobot);

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
u = intvar(numrobot,n,'full');
% dir = intvar(numrobot,n,n,'full'); % a 表示每个机器人的方向
dir_way=intvar(1,num_way,'full');
dir_rob=intvar(numrobot,num_way,'full');


%p = intvar((n-1)/2,1);

% 目标

z=0;
for i = 1:numrobot
    z = z+sum(sum(d.*x(:,:,i)));
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
if l == t
    [N,B]=spread_sin(t,n);
    PATH = [N,B];
    Path(1) = l;
    disp('起点终点在同一个点')
    return
end
%% 约束1 确保路径从起点出发并在终点结束
for i = 1:numrobot
    C = [C, sum(x(l(i),:,i)) - x(l(i),l(i),i) - sum(x(:,t(i),i)) + x(t(i),t(i),i)== 0];

    C = [C, sum(x(l(i),:,i)) - x(l(i),l(i),i) == 1];

    C = [C, sum(x(:,l(i),i)) - x(l(i),l(i),i) - sum(x(t(i),:,i)) + x(t(i),t(i),i) == 0];

    C = [C, sum(x(:,l(i),i)) - x(l(i),l(i),i) == 0]; 
end
%% 约束2 确保出入边条件，每个顶点在路径中仅出现一次
for k=1:numrobot
    for i = 1:n
        if i ~= l(k) && i~=t(k)
            C = [C, sum(x(i,:,k))-x(i,i,k)- sum(x(:,i,k))+x(i,i,k) == 0];
            C = [C, sum(x(i,:,k))-x(i,i,k) <= 1];
            C = [C, sum(x(:,i,k))-x(i,i,k) <= 1];
        end
    end
end
%% 约束3 避免出现子循环
for k=1:numrobot
    for i = 1:n
        for j = 1:n
            if i~=j && i ~=l(k) && i ~=t(k) && j ~=l(k) && j ~=t(k)
                C = [C,u(k,i)-u(k,j) + (n-3)*x(i,j,k)<=n-4];
            end
        end
    end
end

%% 约束4 单行线法则（交叉点不可只出不进或只进不出）
for k = 1:numrobot
for i = 1:m
    [i_x,i_y]=spread_sin(i,size_D);
%     [j_x,j_y]=spread_sin(j,size_D);
    if mod(i,2)==0  && D(i_x,i_y) ~=1 
        if mod(i_x,2)==1 && mod(i_y,2)==0
%             if i+1~=t
                dir_rob(k,i/2) = x(i,i+1,k)+3*x(i+1,i,k);
%             elseif i+1 ==t
%                 dir_rob(k,i/2) = x(k,i-1,i)+3*x(k,i,i-1);
%             end
        end
        if mod(i_x,2)==0 && mod(i_y,2)==1
%             if i+size_D~=t
                dir_rob(k,i/2) = x(i,i+size_D,k)+3*x(i+size_D,i,k);
%             elseif i+size_D==t
%                 dir_rob(k,i/2) = x(k,i-size_D,i)+3*x(k,i,i-size_D);
%             end
        end
    end

end
end
%% 约束5 单行线法则 （巷道方向框定）
   
for k = 1:num_way
%     [i,j]=spread_sin(k,size_D);
        for rob = 1:numrobot
                 C = [ C, max(dir_rob(:,k))-dir_rob(rob,k) ~=2 ];
        end  
end
% %% 在边界处四个交界点：

for k = 1:num_way
     dir_way(k) = max(dir_rob(:,k));
end

for k = 1:num_way
     C=[C, 0<=dir_way(k)<=3];

end

for k =1:num_way
    for k_ro=1:numrobot
         C=[C, 0<=dir_rob(k_ro,k)<=3];
    end
end

% C=[C, 0<=dir_way(:)<=3];
% C=[C, 0<=dir_rob(:,:)<=3];


for k = 1:n
    
    [i,j]=spread_sin(k,size_D);
     if (i==1&&j==1)
        C = [C,  dir_way((j+(i-1)*size_D+1)/2)+dir_way((j+(i)*size_D)/2)~=2];
        C = [C,  dir_way((j+(i-1)*size_D+1)/2)+dir_way((j+(i)*size_D)/2)~=6];
%         C = [C,  dir_way((j+(i-1)*size_D+1)/2)+dir_way((j+(i)*size_D)/2)~=6];
     end
     
     if (i==size_D&&j==size_D)
        C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)~=2];
        C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)~=6];
%         C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)~=6];
     end
     
     if (i==1&&j==size_D)
        C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)~=4];
%         C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)~=2];
%         C = [C,  dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)~=6];
     end
     
     if (i==size_D&&j==1)
        C = [C,  dir_way((j+(i-1-1)*size_D)/2)+dir_way((j+1+(i-1)*size_D)/2)~=4];
%         C = [C,  dir_way((j+(i-1-1)*size_D)/2)+dir_way((j+1+(i-1)*size_D)/2)~=6];
     end
     
end

% 
% 
% %% 周围交界点
for k = 1:n

    [i,j]=spread_sin(k,size_D);
    
    if i==1||i==size_D
        if mod(j,2)==1 && i==1 && j > 1 && j < size_D
           C= [C,  dir_way((j+1+(i-1)*size_D)/2)+ dir_way((j+(i+1-1)*size_D)/2)-dir_way((j-1+(i-1)*size_D)/2) ~=5];
%            C= [C,  dir_way((j+1+(i-1)*size_D)/2)+ dir_way((j+(i+1-1)*size_D)/2)-dir_way((j-1+(i-1)*size_D)/2) ~=-1];
           C= [C,  (dir_way((j+1+(i-1)*size_D)/2)+ dir_way((j+(i+1-1)*size_D)/2))/2*3-max(dir_way((j-1+(i-1)*size_D)/2),1)~=0];
        end
        
        if mod(j,2)==1 && i==size_D && j > 1 && j < size_D
            C= [C, (dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2))/2*3-max(dir_way((j+1+(i-1)*size_D)/2),1) ~=0];
%            C= [C, dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)-dir_way((j+1+(i-1)*size_D)/2)  ~=-1];
           C= [C, dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)-dir_way((j+1+(i-1)*size_D)/2)  ~= 5];
        end
        

    
    elseif mod(i,2)==1 && i>1 && i<size_D
        if j==1
           C= [C, dir_way((j+1+(i-1)*size_D)/2) + dir_way((j+(i-1+1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2) ~=5];
%            C= [C, dir_way((j+1+(i-1)*size_D)/2) + dir_way((j+(i-1+1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2) ~=-1];
           C= [C, (dir_way((j+1+(i-1)*size_D)/2) + dir_way((j+(i-1+1)*size_D)/2))/2*3-max(dir_way((j+(i-1-1)*size_D)/2),1)~=0];
        end
        
        if j==size_D
            C= [C, (dir_way((j-1+(i-1)*size_D)/2) + dir_way((j+(i-1-1)*size_D)/2))/2*3-max(dir_way((j+(i-1+1)*size_D)/2),1)~=0];
%            C= [C, dir_way((j-1+(i-1)*size_D)/2) + dir_way((j+(i-1-1)*size_D)/2)-dir_way((j+(i-1+1)*size_D)/2) ~=-1];
           C= [C, dir_way((j-1+(i-1)*size_D)/2) + dir_way((j+(i-1-1)*size_D)/2)-dir_way((j+(i-1+1)*size_D)/2) ~=5];
        end
    end
    
end
% 
%% 中部点
if numrobot >= 4 % 在机器人个数小于4时中部点不会出现四个方向全为入边或出边的情况

    for k = 1:n
        [i,j]=spread_sin(k,size_D);

        if mod(i,2)==1 &&  i>1 && i < size_D && mod(j,2)==1 && j>1 && j < size_D

%             C = [C, dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)-2*max(dir_way((j-1+(i-1)*size_D)/2),dir_way((j+(i-1+1)*size_D)/2))+dir_way((j+1+(i-1)*size_D)/2)+dir_way((j+(i-1-1)*size_D)/2)-2*max(dir_way((j+(i-1-1)*size_D)/2),dir_way((j+1+(i-1)*size_D)/2))~= 0];

        C = [C, dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2)-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= 4];
%         C = [C, max(dir_way((j-1+(i-1)*size_D)/2),1)+max(dir_way((j+(i-1+1)*size_D)/2),1)-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= -4];
        C = [C, (dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2))*3-(max(dir_way((j+1+(i-1)*size_D)/2),1)+max(dir_way((j+(i-1-1)*size_D)/2),1))~= 0];
        
%         C = [C, (dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2))/2-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= 1];
%         C = [C, (dir_way((j-1+(i-1)*size_D)/2)+dir_way((j+(i-1+1)*size_D)/2))/2-dir_way((j+1+(i-1)*size_D)/2)-dir_way((j+(i-1-1)*size_D)/2)~= -5];
        
        end
    end  

end
  
for k=1:numrobot
    for i = 1:n
        for j = 1:n
            if i==j 
                C = [C,x(i,j,k)==0];
            end
        end
    end
end

%% 求解IP模型

% 参数设置
assign(dir_way,ini_dir_way);

ops = sdpsettings('verbose',1,'solver','gurobi','usex0',1,'gurobi.TimeLimit',timelimit);%verbose计算冗余量，值越大显示越详细
%ops = sdpsettings('verbose',0,'solver','cplex');
% 求解
result  = optimize(C,z,ops);
if result.problem== 0
%    value(z)
%     disp(value(dir_rob))
%    text=' 系统总最优路径长度：';
%    disp([text,num2str(value(z))]);
%     disp('系统总最优路径长度：Best objective');
else
    disp('Finish ! ');
%     disp(value(dir_rob))
end

dir_way_value=value(dir_way);
dir_rob_value=value(dir_rob);
x_value = value(x);
u_value = value(u);

x_add=zeros(n,n);
x_value=cat(3,x_value,x_add);

u_add=zeros(1,n);
u_value=cat(1,u_value,u_add);

dir_rob_add=zeros(1,num_way);
dir_rob_value=cat(1,dir_rob_value,dir_rob_add);


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
    m = size(D,1);
    [X,Y]=spread(Path{i,1},m);
    PATH{i,1}=cat(1,X,Y)'; % 路径存入PATH matrix
end


