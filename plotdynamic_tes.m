function plotdynamic_tes(D,PathStore,Path_num,RobotNum,Start,Goal,~,~)
%AllRobotState = zeros(size(D,1),size(D,2));
MM = size(D,2);
NN = size(D,1);

m=size(D,2);

axis equal;
video = VideoWriter('simulation_fortest','MPEG-4');
video.FrameRate=8;
open(video);

ax = gca();

globaltime = 0;

MAX=0;

temp=0.3;

%%

[PathStore]=biais_goal(PathStore,RobotNum,Start,Goal,m);

[PathStore]=insert_value_dyn(PathStore,RobotNum);

for i = 1:RobotNum
    
    Path_num{i,1}=(PathStore{i,1}(:,2)+(PathStore{i,1}(:,1)-1)*m)';
    
end

for i = 1:RobotNum
    Goal(i)=Path_num{i,1}(length(Path_num{i,1}));
    Start(i)=Path_num{i,1}(1);
end

[X_F,Y_F]=spread(Goal,m);
[X,Y]=spread(Start,m);

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

m_len=size(PathStore{1},1);

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
            
            if x_in==0 && y_in==0
                if j <=m_len-11
                    x_in_end=PathStore{i,1}(j+11,1)-PathStore{i,1}(j,1);
                    y_in_end=PathStore{i,1}(j+11,2)-PathStore{i,1}(j,2);
                    if x_in_end>0 && y_in_end==0
                        Path_dir(i,j)=2;
                    end
                    if y_in_end>0 && x_in_end==0
                        Path_dir(i,j)=1;
                    end
                    if y_in_end==0 && x_in_end<0
                        Path_dir(i,j)=4;
                    end
                    if x_in_end==0 && y_in_end<0
                        Path_dir(i,j)=3;
                    end

                end
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
    pause(0.01);
    cla;
    axis equal;
    xlim([0 MM])
    ylim([0 NN])
    mapdesigner(fliplr(D),1);
    
%     arrowdesign(dir_way,D)
    
    hold on;
%     axis([0 13 0 13]); 
    
    for i=1:RobotNum
        if  ~isempty(PathStore{i,1})
           %AllRobotState(PathStore{i,1}(loop,1),PathStore{i,1}(loop,2)) = 1;
           if PathStore{i,1}(loop,1)==X_F(i) && PathStore{i,1}(loop,2)==Y_F(i)
               axis([0,MM,0,NN])
                axis equal
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10)  %一般情况下机器人不会中途经过终点    
                xlim([0 MM])
                ylim([0 NN])
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
            line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',5);
    
           elseif PathStore{i,1}(loop,1)==X(i) && PathStore{i,1}(loop,2)==Y(i) 
               if loop==1
                axis([0,MM,0,NN])   
                axis equal
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','y','MarkerFaceColor','y','MarkerSize',10)
                xlim([0 MM])
                ylim([0 NN])
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
                
                line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',5);
                
               else
                axis([0,MM,0,NN])
                axis equal
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)   %中途经过起点
                xlim([0 MM])
                ylim([0 NN])
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
                line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',5);
            
               end
                
           else
              axis([0,MM,0,NN])
               axis equal
                plot(PathStore{i,1}(loop,2)-1/2,PathStore{i,1}(loop,1)-1/2,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)
                xlim([0 MM])
                ylim([0 NN])
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
            line([x,xx],[y,yy],'color','k','linestyle','-','lineWidth',5);
            
           end
        end
    end
  
    writeVideo(video,getframe(ax))
    globaltime = globaltime + 1;  
end

% disp('系统总消耗时刻：')
% disp(globaltime);
%disp(loop);
% close(gca);
close(video);

% for i = 1:RobotNum
%     disp(Path_num{i,1}(end)-Goal(i))
% end