%% Version: 02/02/2020
% Author:HUO JIAXI
% 已完成单机器人的IP模型，并且已实现较小规模的路径规划问题求解
% 后期返回D matrix， PATH matrix
% 输入机器人i的起点l和终点t X行 Y列
%% 待修改
function [RE,PATH,Path]=IP_solver_op(D,l,t,numrobot)
d = transfer(D);
n = size(d,1); 
% 决策变量
x = binvar(n,n,'full'); % n*n维的决策变量
u = intvar(1,n);
% 目标
z = sum(sum(d.*x));

% 约束添加
C = [];
%%后期作为参数在调用中赋值
%%
% 静止不动时返回0
if l == t
    PATH = [];
    Path(1) = l;
    distance = 0;
    disp('起点终点不能选择同一个点')
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
%% 求解IP模型

% 参数设置

ops = sdpsettings('verbose',0,'solver','gurobi');%verbose计算冗余量，值越大显示越详细

% 求解
result  = optimize(C,z,ops);

RE=0;

if result.problem== 0
%    value(x)
%    value(z)
    text=' 号机器人最优路径求解成功，最优路径长度：';
    disp([num2str(numrobot),text,num2str(value(z))]);
else
    RE=1;
    disp('求解路径中存在障碍物，起点终点无法直达');
end

if RE == 0
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

    Path=solvermatrix(o,l,t);

    m = size(D,1);
    [X,Y]=spread(Path,m);
    PATH=cat(1,X,Y)'; % 路径存入PATH matrix
    RE=0;
else
    PATH = [];
    Path = [];
    RE=1;
end

