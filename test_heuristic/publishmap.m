D = load('tsp_dist_broad.txt'); 
mapdesigner_global(fliplr(D),2);

[Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);

mapdesigner_global(fliplr(D_reduit),2);
axis equal
xlim([0,size(D_reduit,2)])
ylim([0,size(D_reduit,1)])