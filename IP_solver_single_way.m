%% Version: 28/02/2020
% Author:HUO JIAXI
% 引入单行道限制
%% 待修改
function [PATH,Path]=IP_solver_single_way(D,l,t,numrobot,size_D)

d = transfer(D);
n = size(d,1); 
% 决策变量
x = binvar(n,n,'full'); % n*n维的决策变量
u = intvar(1,n);
dir = intvar(n,n); % a 表示每个巷道的方向
% 目标
z = sum(sum(d.*x));

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
C = [C, sum(x(l,:)) - x(l,l) - sum(x(:,t)) + x(t,t)== 0];

C = [C, sum(x(l,:)) - x(l,l) == 1];

C = [C, sum(x(:,l)) - x(l,l) - sum(x(t,:)) + x(t,t) == 0];

C = [C, sum(x(:,l)) - x(l,l) == 0]; 
%% 约束2 确保出入边条件，每个顶点在路径中仅出现一次

for i = 1:n
    if i ~= l && i~=t
        C = [C, sum(x(i,:))-x(i,i)- sum(x(:,i))+x(i,i) == 0];
        C = [C, sum(x(i,:))-x(i,i) <= 1];
        C = [C, sum(x(:,i))-x(i,i) <= 1];
    end
end

%% 约束3 避免出现子循环

for i = 1:n
    for j = 1:n
        if i~=j && i ~=l && i ~=t && j ~=l && j ~=t
            C = [C,u(i)-u(j) + (n-3)*x(i,j)<=n-4];
        end
    end
end

%% 约束4 单行线法则
for i = 1:n
   for j = 1:n
    [i_x,i_y]=spread_sin(i,size_D);
%    [j_y,j_y]=spread_sin(j,size_D);
    if i > j && (mod(i,2)==0 || mod(j,2)==0) && (abs(i-j)== 1 || abs(i-j)== size_D) && D(i_x,i_y) ~=1
        dir(i,j) = x(i,j);
    elseif i < j && (mod(i,2)==0 || mod(j,2)==0) && (abs(i-j)== 1 || abs(i-j)== size_D) && D(i_x,i_y) ~=1
        dir(i,j) = -x(i,j);
    elseif i == j && mod(i,2)==0
        dir(i,j) = 0;  
    else
        dir(i,j) = 0;
    end    
   end
end

for i = 1:n
            C = [C, sum(dir(i,:))+sum(dir(:,i))<=1];
end


%% 
%% 求解IP模型

% 参数设置

ops = sdpsettings('verbose',0,'solver','gurobi');%verbose计算冗余量，值越大显示越详细

% 求解
result  = optimize(C,z,ops);
if result.problem== 0
%    value(x)
%    disp(value(z))
    text=' 号机器人最优路径求解成功，最优路径长度：';
    disp([num2str(numrobot),text,num2str(value(z))]);
else
    disp('求解路径中存在障碍物，起点终点无法直达');
end

disp(value(dir))
o=value(x);

for i=1:length(o)
    for j = 1 : length(o)
        if(i==j)
            o(i,j)=0;
        else
            continue;
        end
    end
end

%% 邻接矩阵转换，路径绘制

Path=solvermatrix(o,l,t);

m = size(D,1);
[X,Y]=spread(Path,m);
PATH=cat(1,X,Y)'; % 路径存入PATH matrix







