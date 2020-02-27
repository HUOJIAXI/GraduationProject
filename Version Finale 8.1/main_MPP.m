% Final projet : Intergrate Programming as a path-finding methode for MAS
% Author : HUO JIAXI
% Version : 17/01/2020

%% Initialize
clear;
clc;

% Define the size of Map
xlength = 10;
ylength = 13;
robotNum = 0; % 机器人个数
podNum = 60; % 货架个数
depotNum = 0; % 仓储点个数
taskNum = robotNum; % 规定任务个数与机器人个数相同

xy2rc=@(x,y)[ylength+1-y;x];
rc2xy=@(r,c)[c;ylength+1-r];

%globalTime = 1;

%% MAP
[AllRobotState,AllPodState,RobotOccupancy,PodOccupancy,MapOccupancy]=drawMap(xlength,ylength,robotNum,podNum,depotNum);

num = 0;
numl=1;
obs=[];
D = transfer(MapOccupancy);
[r,c] = size(MapOccupancy);
% for i = 1:r
%     for k = 1:c
%         num = num + 1;
%         if MapOccupancy(i,k)==1
%             obs(numl) = num;
%             numl = numl + 1;
%         end
%             
%     end
% end
% 
% [m,z]=size(D);
% for i = 1:m
%     for k = 1:numl-1
%             if i==obs(k)
%                 D(i,:)=3;
%             end
%     end
% end
% 
% for j = 1:z
%     for k = 1:numl-1
%             if j==obs(k)
%                 D(:,j)=3;
%             end
%     end
% end
% 
% id = D(:,1) == 3; 
% D(id,:)=[];
% 
% id = D(1,:) == 3; 
% D(:,id)=[]; 
%将领接矩阵中含有货架的行列删除，

% [r,c]=size(D);
% for i = 1:r
%     for j = 1:c
%         if D(i,j)==1 || D(i,j)==0 ||D(i,j)==inf
%             continue;
%         else
%             D(i,j)=i+j;
%         end
%         
%     end
% end


%%

% n = size(D,1); 
% 
% %决策变量
% x = binvar(n,n,'full'); % n*n维的决策变量
% u = intvar(1,n);
% % 目标
% z = sum(sum(D.*x));
% 
% % 约束添加
% C = [];
% l=1;
% t=20;
% 
% C = [C, sum(x(l,:)) - x(l,l) - sum(x(:,t)) + x(t,t)== 0];
% %s = sum(x(l,:)) - x(l,l);
% C = [C, sum(x(l,:)) - x(l,l) == 1];
% %s = sum(x(:,t)) - x(t,t) - sum(x(t,:)) + x(t,t);
% C = [C, sum(x(:,l)) - x(l,l) - sum(x(t,:)) + x(t,t) == 0];
% %s = sum(x(:,l)) - x(l,l);
% C = [C, sum(x(:,l)) - x(l,l) == 0];     
% 
% for i = 1:n
%     if i ~= l && i~=t
%         C = [C, sum(x(i,:))-x(i,i)- sum(x(:,i))+x(i,i) == 0];
%         C = [C, sum(x(i,:))-x(i,i) <= 1];
%         C = [C, sum(x(:,i))-x(i,i) <= 1];
%     end
% end
% 
% for i = 1:n
%     for j = 1:n
%         if i~=j
%             C = [C,u(i)-u(j) + (n-3)*x(i,j)<=n-4];
%         end
%     end
% end
% % 参数设置
% 
% ops = sdpsettings('verbose',3,'solver','cplex');%verbose计算冗余量，值越大显示越详细
% %ops = sdpsettings('verbose',3)
% 
% % 求解
% result  = optimize(C,z,ops);
% if result.problem== 0
%     value(x)
%     value(z)
% else
%     disp('求解过程中出错');
% end


