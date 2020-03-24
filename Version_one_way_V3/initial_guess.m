function [ini_x_value]=initial_guess(ini_x_value,Start,Goal,D)
size_x=size(D,1);
size_y=size(D,2);
size_total=size_x*size_y;

[~,Path_num_sin] = ori_path_am(Start,Goal,D);

sup_ini=zeros(size_total,size_total);

for i = 1:length(Path_num_sin)-1
    sup_ini(Path_num_sin(i),Path_num_sin(i+1))=1;
end

ini_x_value=cat(3,ini_x_value,sup_ini);

