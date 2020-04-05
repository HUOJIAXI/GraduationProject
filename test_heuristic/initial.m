function [ini_dir_way] = initial(D)
size_m=size(D,1);
size_n=size(D,2);
ini_dir_way=[];
for i = 1:size_m
     if mod(i,2) == 1 && mod((i+1)/2,2)==0
       for j = 1:size_n 

            if mod(j/2,2)==1
              ini_dir_way=[ini_dir_way 3];
            elseif mod(j/2,2)==0
              ini_dir_way=[ini_dir_way 1]; 
            else
              ini_dir_way=[ini_dir_way 0]; 
            end

       end
     end
     
     if mod(i,2) == 1 && mod((i+1)/2,2)==1
       for j = 1:size_n 

            if mod(j/2,2)==1
              ini_dir_way=[ini_dir_way 1];
            elseif mod(j/2,2)==0
              ini_dir_way=[ini_dir_way 3]; 
            else
              ini_dir_way=[ini_dir_way 0]; 
            end

       end
     end
     
     if mod(i,2) == 0 && mod(i/2,2)==1
         for j = 1:size_n
             if mod((j+1)/2,2)==1
                 ini_dir_way=[ini_dir_way 3];
             
             elseif mod((j+1)/2,2)==0
                 ini_dir_way=[ini_dir_way 1];
             else
                 ini_dir_way=[ini_dir_way 0]; 
            end
         end
          
     end
     
      if mod(i,2) == 0 && mod(i/2,2)==0
         for j = 1:size_n
             if mod((j+1)/2,2)==1
                 ini_dir_way=[ini_dir_way 1];           
             elseif mod((j+1)/2,2)==0
                 ini_dir_way=[ini_dir_way 3];
             else
                 ini_dir_way=[ini_dir_way 0]; 
            end
         end
          
      end
     
end

% ini_dir_way=[1 3 1 3 1 3 1 3 1 3 1 3 1 3 1 3 1 3 1 3 1 3 1 3];