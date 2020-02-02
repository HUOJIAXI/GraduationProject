% 利用yalmip调用cplex求解器求解TSP问题
clear;clc;close all;
D = load('tsp_map.txt')';
mapdesigner(D);
hold on;
d = transfer(D);
n = size(d,1); 
% 决策变量
x = binvar(n,n,'full'); % n*n维的决策变量
u = intvar(1,n);
% 目标
z = sum(sum(d.*x));

% 约束添加
C = [];
l=2;
t=18;
if l == t
    Path = [];
    Path(1) = l;
    distance = 0;
    return
end

C = [C, sum(x(l,:)) - x(l,l) - sum(x(:,t)) + x(t,t)== 0];

C = [C, sum(x(l,:)) - x(l,l) == 1];

C = [C, sum(x(:,l)) - x(l,l) - sum(x(t,:)) + x(t,t) == 0];

C = [C, sum(x(:,l)) - x(l,l) == 0];     

for i = 1:n
    if i ~= l && i~=t
        C = [C, sum(x(i,:))-x(i,i)- sum(x(:,i))+x(i,i) == 0];
        C = [C, sum(x(i,:))-x(i,i) <= 1];
        C = [C, sum(x(:,i))-x(i,i) <= 1];
    end
end

for i = 1:n
    for j = 1:n
        if i~=j && i ~=l && i ~=t && j ~=l && j ~=t
            C = [C,u(i)-u(j) + (n-3)*x(i,j)<=n-4];
        end
    end
end

% 参数设置

ops = sdpsettings('verbose',0,'solver','cplex');%verbose计算冗余量，值越大显示越详细

% 求解
result  = optimize(C,z,ops);
if result.problem== 0
    value(x)
    value(z)
else
    disp('求解过程中出错');
end

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

% if l < t
%     G = graph(o,'upper','OmitSelfLoops');  
% end
% if l > t
%     G = graph(o,'lower','OmitSelfLoops');
% end
% k=G.Edges;
% 
% [Path,distance] = shortestpath(G,l,t,'Method','positive');
%if l < t
Path=solvermatrix(o,l,t);
%end
%if l > t
%    Path=solvermatrix(o,l,t,1);
%end
[X,Y]=spread(Path);
PATH=cat(1,X,Y)';
plot(X-1/2,Y-1/2,'-ks','MarkerFaceColor','r','MarkerSize',10)
hold on;
%h=annotation('arrow',[(X(2)-1/2)/10 (X(3)-1/2)/10],[(Y(2)-1/2)/10 (Y(3)-1/2)/10]) ;
%m=G.Edges;
A=zeros(size(d,1),size(d,2));
for i =2:length(Path)
    A(Path(i-1),Path(i))=1;
end
if l<t
    A(size(A,2),:)=0;
end

if l>t
    A(:,size(A,1))=0;
end

G=digraph(A,'OmitSelfLoops')
figure(2);
plot(G);



