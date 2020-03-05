%% Version: 28/02/2020
% Author:HUO JIAXI
% 引入单行道限制
%% 待修改
function [PATH,Path]=IP_solver_single_way_dev(D,l,t,numrobot,size_D)
PATH=cell(numrobot,1);
Path=cell(numrobot,1);
o_single=cell(numrobot,1);

d = transfer(D);
n = size(d,1); 
% 决策变量

x = binvar(n,n,numrobot,'full'); % n*n维的决策变量
u = intvar(numrobot,n);
dir = intvar(n,n,numrobot,'full'); % a 表示每个巷道的方向

% 目标
z = sum(sum(d.*x(:,:,1)));

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
    C = [C, sum(x(l(i),:,i))-x(l(i),l,i)-sum(x(:,t,i))+x(t,t,i)==0];

    C = [C, sum(x(l,:,i)) - x(l,l,i) == 1];

    C = [C, sum(x(:,l,i)) - x(l,l,i) - sum(x(t,:,i)) + x(t,t,i) == 0];

    C = [C, sum(x(:,l,i)) - x(l,l,i) == 0]; 
end
%% 约束2 确保出入边条件，每个顶点在路径中仅出现一次
for k=1:numrobot
    for i = 1:n
        if i ~= l(k) && i~=t(k)
            C = [C, sum(x(k,i,:))-x(k,i,i)- sum(x(k,:,i))+x(k,i,i) == 0];
            C = [C, sum(x(k,i,:))-x(k,i,i) <= 1];
            C = [C, sum(x(k,:,i))-x(k,i,i) <= 1];
        end
    end
end
%% 约束3 避免出现子循环
for k=1:numrobot
    for i = 1:n
        for j = 1:n
            if i~=j && i ~=l && i ~=t && j ~=l && j ~=t
                C = [C,u(k,i)-u(k,j) + (n-3)*x(k,i,j)<=n-4];
            end
        end
    end
end

%% 约束4 单行线法则
for k = 1:numrobot
    for i = 1:n
       for j = 1:n
        [i_x,i_y]=spread_sin(i,size_D);
    %    [j_y,j_y]=spread_sin(j,size_D);
        if i > j && (mod(i,2)==0 || mod(j,2)==0) && (abs(i-j)== 1 || abs(i-j)== size_D) && D(i_x,i_y) ~=1
            dir(k,i,j) = x(k,i,j);
        elseif i < j && (mod(i,2)==0 || mod(j,2)==0) && (abs(i-j)== 1 || abs(i-j)== size_D) && D(i_x,i_y) ~=1
            dir(k,i,j) = -x(k,i,j);
        elseif i == j && mod(i,2)==0
            dir(k,i,j) = 0;  
        else
            dir(k,i,j) = 0;
        end    
       end
    end
end

for k = 1:numrobot
    for i = 1:n
        if i==1||i==3||i==6||i==9
                C = [C, sum(dir(k,i,:))+sum(dir(k,:,i))<=1];
        end
    end
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
%disp(o);
for k=1:numrobot
    for i=1:length(o)
        for j = 1 : length(o)
            if(i==j)
                o(k,i,j)=0;
            else
                continue;
            end
        end
    end
end
%% 邻接矩阵转换，路径绘制

for i = 1:numrobot
    o_single{i,1} = reshape(o, n,n*i); 
end

for i = 1:numrobot
    o_sin=o_single{i,1};
    disp(o_sin)
    Path{i,1}=solvermatrix(o_sin,l,t);
    m = size(D,1);
    [X,Y]=spread(Path{i,1},m);
    PATH{i,1}=cat(1,X,Y)'; % 路径存入PATH matrix
end








