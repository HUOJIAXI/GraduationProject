%% Version: 28/02/2020
% Author:HUO JIAXI
% 引入单行道限制
%% 待修改
function [PATH,Path]=IP_solver_single_way(D,l,t,numrobot,size_D)
PATH=cell(numrobot,1);
Path=cell(numrobot,1);
o_single=cell(numrobot,1);

d = transfer(D);
n = size(d,1); 
% 决策变量

x = binvar(numrobot,n,n,'full'); % n*n维的决策变量
u = intvar(numrobot,n);
dir = intvar(numrobot,n,n,'full'); % a 表示每个巷道的方向

% 目标
z=0;
for i = 1:numrobot
    z = z+sum(sum(d.*x(i,:,:)));
end
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
    C = [C, sum(x(i,l(i),:)) - x(i,l(i),l(i)) - sum(x(i,:,t(i))) + x(i,t(i),t(i))== 0];

    C = [C, sum(x(i,l(i),:)) - x(i,l(i),l(i)) == 1];

    C = [C, sum(x(i,:,l(i))) - x(i,l(i),l(i)) - sum(x(i,t(i),:)) + x(i,t(i),t(i)) == 0];

    C = [C, sum(x(i,:,l(i))) - x(i,l(i),l(i)) == 0]; 
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
            if i~=j && i ~=l(k) && i ~=t(k) && j ~=l(k) && j ~=t(k)
                C = [C,u(k,i)-u(k,j) + (n-3)*x(k,i,j)<=n-4];
            end
        end
    end
end

%% 约束4 单行线法则（交叉点不可只出不进或只进不出）
for k = 1:numrobot
    for i = 1:n
       for j = 1:n
        [i_x,i_y]=spread_sin(i,size_D);
    %    [j_y,j_y]=spread_sin(j,size_D);
        if i > j && (mod(i,2)==0 || mod(j,2)==0) && (abs(i-j)== 1 || abs(i-j)== size_D) && D(i_x,i_y) ~=1
            dir(k,i,j) = x(k,i,j); % 由大索引到小索引的方向为1
        elseif i < j && (mod(i,2)==0 || mod(j,2)==0) && (abs(i-j)== 1 || abs(i-j)== size_D) && D(i_x,i_y) ~=1
            dir(k,i,j) = x(k,i,j)+1; % 由小索引到大索引的方向为2
        elseif i == j && mod(i,2)==0
            dir(k,i,j) = 0;  
        else
            dir(k,i,j) = 0;
        end    
       end
    end
end

for k = 1:numrobot-1
    for i = 1:size_D
        if mod(i,2)==1
            for j = 1:size_D
                if mod(j,2)==1
                    C = [C, sum(dir(k,j+(i-1)*size_D,:))+sum(dir(k+1,j+(i-1)*size_D,:))~=2];
                    C = [C, sum(dir(k,j+(i-1)*size_D,:))+sum(dir(k+1,j+(i-1)*size_D,:))~=4];
                end
            end
        end
    end
end
%% 约束5 单行线法则 （巷道方向框定）

% for i = 1:n
%     for j = 1:n
%         C = [C, dir(1,1,2)-dir(2,3,2)+dir(2,8,7)+dir(1,7,8)==0]; %% 仅对于3*3小环境有效
%     end
% end
for k = 1:numrobot-1
    for i = 1:n
        for j = 1:n
                C=[C,dir(k,i,j)+dir(k+1,j,i) ~= 3 ];
        end
    end
end

%% 

for k=1:numrobot
    for i = 1:n
        for j = 1:n
            if i==j 
                C = [C,x(k,i,j)==0];
            end
        end
    end
end

%% 求解IP模型

% 参数设置

ops = sdpsettings('verbose',1,'solver','gurobi');%verbose计算冗余量，值越大显示越详细
%ops = sdpsettings('verbose',0,'solver','cplex');
% 求解
result  = optimize(C,z,ops);
if result.problem== 0
%    value(z)
%    disp(value(x))
%    text=' 系统总最优路径长度：';
%    disp([text,num2str(value(z))]);
    disp('系统总最优路径长度：Best objective');
else
    disp('求解路径中存在障碍物，起点终点无法直达');
end

%disp(value(dir))
o=value(x);

% for k=1:numrobot
%     for i=1:length(o)
%         for j = 1 : length(o)
%             if(i==j)
%                 o(k,i,j)=0;
%             else
%                 continue;
%             end
%         end
%     end
% end

%disp(o(2,:,:));
%% 邻接矩阵转换，路径绘制

for i = 1:numrobot
    o_single{i,1} =  squeeze(o(i,:,:)) ; 
end

for i = 1:numrobot
    o_sin=o_single{i,1};
%    disp(o_sin)
    Path{i,1}=solvermatrix(o_sin,l(i),t(i));
    m = size(D,1);
    [X,Y]=spread(Path{i,1},m);
    PATH{i,1}=cat(1,X,Y)'; % 路径存入PATH matrix
end








