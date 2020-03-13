function plotdynamic(D,PathStore,Path_num,RobotNum,Start,Goal)
%AllRobotState = zeros(size(D,1),size(D,2));
m=size(D,1);
n=size(D,2);
[X,Y]=spread(Start,m);
[X_F,Y_F]=spread(Goal,m);
video = VideoWriter('simulation_16ROB_COLI_version8.1','MPEG-4');
video.FrameRate=2;
open(video);

ax = gca();

globaltime = 0;

%% 明确障碍物位置
nobs=[];
obs=[];
for i = 1:m
    for j = 1:n
        if D(i,j) == 1
            obs=[obs j+(i-1)*n];
%             [obs_x,obs_y]=spread(obs,m);
        else
            nobs=[nobs j+(i-1)*n];
        end
    end
end
%%



MAX=0;

for i = 1:RobotNum
%     [PATH,path_num]=IP_solver(D,Start(i),Goal(i),i);
%     PathStore{i} = PATH;
%     Path_num{i} = path_num;
    MAX=max([size(PathStore{i,1},1),MAX]);
%    MAX=MAX+1;
    H(i)=size(PathStore{i,1},1);
end

MAX = MAX+1;

for i=1:RobotNum
    if size(PathStore{i},1)<MAX 
        SI=size(PathStore{i});
        for j = (SI+1):MAX
            PathStore{i}(j,1) =  PathStore{i}(H(i),1);
            PathStore{i}(j,2) =  PathStore{i}(H(i),2);
%             Path_num{i}(j)=Path_num{i}(H(i));
        end
    end
    
end

for i=1:RobotNum
    final = unique(find(Path_num{i,1}==Goal(i)));
    finalindice(i) = final(1);
end

indice=max(finalindice);

for loop=1:10000
    if loop > indice+1
        break;
    end

    pause(0.5);
    cla;
    
    mapdesigner(fliplr(D),1);
    hold on;
%     axis([0 13 0 13]); 
    
    for i=1:RobotNum
        if  ~isempty(PathStore{i,1})
           %AllRobotState(PathStore{i,1}(loop,1),PathStore{i,1}(loop,2)) = 1;
           if find(nobs==Path_num{i,1}(loop))
               if PathStore{i,1}(loop,1)==X_F(i) && PathStore{i,1}(loop,2)==Y_F(i)
                    plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10)  %一般情况下机器人不会中途经过终点
               elseif PathStore{i,1}(loop,1)==X(i) && PathStore{i,1}(loop,2)==Y(i)
                   if loop==1
                    plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','y','MarkerFaceColor','y','MarkerSize',10)
                   else
                    plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)   %中途经过起点
                   end

               else
                    plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)
               end
           else
               plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',10) 
           end
        end
    end
    
%     plot(AllRobotState-1/2,'ks','MarkerFaceColor','r','MarkerSize',10)
    
%     for i=1:RobotNum
%         if  ~isempty(PathStore{i,1})
%            AllRobotState(PathStore{i,1}(loop,1),PathStore{i,1}(loop,2)) = 0;
%         end
%     end
    writeVideo(video,getframe(ax))
    globaltime = globaltime + 1;

end

disp('系统总消耗时刻：')
disp(globaltime);
disp(loop);
close(video);