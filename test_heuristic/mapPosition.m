D = load('tsp_dist_broad.txt'); 
mapdesigner_global(fliplr(D),2);

% [Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);
% 
% mapdesigner_global(fliplr(D),2);
axis equal
xlim([0,size(D,2)])
ylim([0,size(D,1)])
plot(8.5,7.5,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)

x=8.5;
y=7.5;
temp=0.3;
switch 4
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

hold on

plot(3.5,4.5,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10)

x=3.5;
y=4.5;
temp=0.3;
switch 1
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
