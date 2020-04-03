function [X_fin,Y_fin] = spread_sin(Goal,SD)
if mod(Goal,SD)==0
    Y_fin = SD;
    X_fin = Goal/SD;
else
    Y_fin = mod(Goal,SD);
    X_fin = floor(Goal/SD)+1;
end