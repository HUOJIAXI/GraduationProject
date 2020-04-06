function [ini_x_value]=initial_guess_heuristic(ini_Path_num,ini_x_value,D)
size_x=size(D,1);
size_y=size(D,2);
size_total=size_x*size_y;

sup_ini=zeros(size_total,size_total);

for i = 1:length(ini_Path_num)-1
    sup_ini(ini_Path_num(i),ini_Path_num(i+1))=1;
end

ini_x_value=cat(3,ini_x_value,sup_ini);