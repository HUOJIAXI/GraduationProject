function [D]=construct_D(max)

D = zeros(max,max);
for i = 1:max
    if mod(i,2)==1
        continue
    end
    
    if mod(i,2)==0
        for j = 1:max
            if mod(j,2)==0
                D(i,j) = 1;
            end
        end
    end
end