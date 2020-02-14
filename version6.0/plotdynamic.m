function plotdynamic(D,PathStore,Path_num,RobotNum,Start,Goal)
%AllRobotState = zeros(size(D,1),size(D,2));
m=size(D,1);
[X,Y]=spread(Start,m);
[X_F,Y_F]=spread(Goal,m);
video = VideoWriter('simulation_8ROB_COLI_version6.0','MPEG-4');
video.FrameRate=2;
open(video);
globaltime = 0;

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


for loop=1:50
    if loop > indice+1
        break;
    end
    frame = getframe;
    writeVideo(video,frame);
    pause(0.5);
    cla;
    
    mapdesigner(fliplr(D),1);
    hold on;
%     axis([0 13 0 13]); 
    
    for i=1:RobotNum
        if  ~isempty(PathStore{i,1})
           %AllRobotState(PathStore{i,1}(loop,1),PathStore{i,1}(loop,2)) = 1;
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
        end
    end
    
%     plot(AllRobotState-1/2,'ks','MarkerFaceColor','r','MarkerSize',10)
    
%     for i=1:RobotNum
%         if  ~isempty(PathStore{i,1})
%            AllRobotState(PathStore{i,1}(loop,1),PathStore{i,1}(loop,2)) = 0;
%         end
%     end
    globaltime = globaltime + 1;
end

disp('系统总消耗时刻：')
disp(globaltime);
disp(loop);

close(video);