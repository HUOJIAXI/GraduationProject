function plotdynamic(D,PathStore,RobotNum,Start,Goal)
%AllRobotState = zeros(size(D,1),size(D,2));
m=size(D,1);
[X,Y]=spread(Start,m);
[X_F,Y_F]=spread(Goal,m);
video = VideoWriter('simulation_6ROB_COLI','MPEG-4');
video.FrameRate=2;
open(video);
globaltime = 0;
for loop=1:50
    if loop > size(PathStore{1,1},1)
        break;
    end
    frame = getframe;
    writeVideo(video,frame);
    pause(0.5);
    cla;
    
    mapdesigner(fliplr(D),1);
    hold on;
    
    for i=1:RobotNum
        if  ~isempty(PathStore{i,1})
           %AllRobotState(PathStore{i,1}(loop,1),PathStore{i,1}(loop,2)) = 1;
           if PathStore{i,1}(loop,1)==X_F(i) && PathStore{i,1}(loop,2)==Y_F(i)
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10)
                
           elseif PathStore{i,1}(loop,1)==X(i) && PathStore{i,1}(loop,2)==Y(i)   
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','y','MarkerFaceColor','y','MarkerSize',10)
                
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

close(video);