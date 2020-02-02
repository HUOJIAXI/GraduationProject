function [X,Y]=spread(Path)
for i = 1:length(Path)
    if mod(Path(i),5)==0
        X(i) = 5;
        Y(i) = Path(i)/5;
    else
        X(i) = mod(Path(i),5);
        Y(i) = floor(Path(i)/5)+1;
    end
end

