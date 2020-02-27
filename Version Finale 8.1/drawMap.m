function [AllRobotState,AllPodState,RobotOccupancy,PodOccupancy,MapOccupancy]=drawMap(xlength,ylength,robotNum,podNum,depotNum)

%xy2rc=@(x,y)[ylength+1-y;x];
%rc2xy=@(r,c)[c;ylength+1-r];

sz=get(0,'screensize');
sz(1,2) = 80;
sz(1,4) = 950;
h=figure('outerposition',sz);
assignin('base','h',h); %in case of any callback errors.
hold on;
grid on;
set(gca,'xtick',0:1:xlength);
set(gca,'ytick',0:1:ylength);
axis equal;
axis([0 xlength+1 0 ylength+1]);
axis manual;

% stores current states
AllRobotState = zeros(robotNum,3);
AllPodState = zeros(podNum,3);

MapOccupancy = zeros(ylength,xlength);
RobotOccupancy = zeros(ylength,xlength);

[RobotStates,PodStates,DepotStates,StorageOccupancy]=initialize(xlength,ylength,robotNum,podNum,depotNum);
PodOccupancy = StorageOccupancy;

MapOccupancy = MapOccupancy+PodOccupancy;

rectangle('Position', [0,0,xlength+1,ylength+1],'lineWidth',5);
plot(DepotStates(:,1),DepotStates(:,2),'square','MarkerEdgeColor',[0.5 0.5 0.5],'MarkerFaceColor',[0.7 0.7 0.7],'MarkerSize',30);
plot(PodStates(:,1),PodStates(:,2),'square','MarkerEdgeColor','k','MarkerFaceColor',[1 1 1],'MarkerSize',60);
plot(RobotStates(:,1),RobotStates(:,2),'o','MarkerEdgeColor','k','MarkerFaceColor',[1 0 0],'MarkerSize',15);