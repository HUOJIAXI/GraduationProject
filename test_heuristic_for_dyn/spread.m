function [X,Y]=spread(Path,m)
for i = 1:length(Path)
    if mod(Path(i),m)==0
        Y(i) = m;
        X(i) = Path(i)/m;
    else
        Y(i) = mod(Path(i),m);
        X(i) = floor(Path(i)/m)+1;
    end
end

