function plotdynamic(D,PathStore,Path_num,RobotNum,Start,Goal)
%AllRobotState = zeros(size(D,1),size(D,2));
m=size(D,2);
[X,Y]=spread(Start,m);
[X_F,Y_F]=spread(Goal,m);
video = VideoWriter('simulation_16ROB_Version5.0','MPEG-4');
video.FrameRate=2;
open(video);

ax = gca();

globaltime = 0;

MAX=0;

temp=0.1;

%%

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

%% 辨别方向
for i = 1:RobotNum
        for j =1:MAX-1
            x_in=PathStore{i,1}(j+1,1)-PathStore{i,1}(j,1);
            y_in=PathStore{i,1}(j+1,2)-PathStore{i,1}(j,2);
            if x_in>=0 && y_in==0
                Path_dir(i,j)=2;
            end
            if y_in>=0 && x_in==0
                Path_dir(i,j)=1;
            end
            if y_in==0 && x_in<=0
                Path_dir(i,j)=4;
            end
            if x_in==0 && y_in<=0
                Path_dir(i,j)=3;
            end
            
            if j == MAX-1
                Path_dir(i,j+1)=Path_dir(i,j);
            end
        end
end


for i=1:RobotNum
    test=Path_num{i}-Goal(i);
    final=find(test==0);
%     final = length(Path_num{i});
    finalindice(i) = final(1);
end

indice=max(finalindice);
%disp(indice)

for loop=1:10000
    if loop > indice+1
        break;
    end
%     frame = getframe(ax);
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
                
                x=PathStore{i,1}(loop,2)-1/2;
                y=PathStore{i,1}(loop,1)-1/2;
                
                switch Path_dir(i,loop)
                case 1
                    xx = x+temp;
                    yy = y;
                case 2
                    xx = x;
                    yy = y+temp;
                case 3
                    xx = x-temp;
                    yy = y;
                case 4
                    xx = x;
                    yy = y-temp;            
                end
            line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',3);
    
           elseif PathStore{i,1}(loop,1)==X(i) && PathStore{i,1}(loop,2)==Y(i) 
               if loop==1
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','y','MarkerFaceColor','y','MarkerSize',10)
                
                x=PathStore{i,1}(loop,2)-1/2;
                y=PathStore{i,1}(loop,1)-1/2;
                
                switch Path_dir(i,loop)
                case 1
                    xx = x+temp;
                    yy = y;
                case 2
                    xx = x;
                    yy = y+temp;
                case 3
                    xx = x-temp;
                    yy = y;
                case 4
                    xx = x;
                    yy = y-temp;            
                end
                
                line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',3);
                
               else

                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)   %中途经过起点
                x=PathStore{i,1}(loop,2)-1/2;
                y=PathStore{i,1}(loop,1)-1/2;
                switch Path_dir(i,loop)
                case 1
                    xx = x+temp;
                    yy = y;
                case 2
                    xx = x;
                    yy = y+temp;
                case 3
                    xx = x-temp;
                    yy = y;
                case 4
                    xx = x;
                    yy = y-temp;            
                end
                line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',3);
            
               end
                
           else
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)
                
                x=PathStore{i,1}(loop,2)-1/2;
                y=PathStore{i,1}(loop,1)-1/2;
                switch Path_dir(i,loop)
                case 1
                    xx = x+temp;
                    yy = y;
                case 2
                    xx = x;
                    yy = y+temp;
                case 3
                    xx = x-temp;
                    yy = y;
                case 4
                    xx = x;
                    yy = y-temp;            
                end
            line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',3);
            
           end
        end
    end
  
    writeVideo(video,getframe(ax))
    globaltime = globaltime + 1;  
end

disp('系统总消耗时刻：')
disp(globaltime);
%disp(loop);
% close(gca);
close(video);

% for i = 1:RobotNum
%     disp(Path_num{i,1}(end)-Goal(i))
% end