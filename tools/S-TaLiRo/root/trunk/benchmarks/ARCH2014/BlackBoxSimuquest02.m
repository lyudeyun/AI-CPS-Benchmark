function [T XT YT LT CLG Guards] = BlackBoxSimuquest02(X0,simT,TU,U)

simopt = simget('bardh_i4_demo_harness_PFI_02');
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YTtmp] = sim('bardh_i4_demo_harness_PFI_02',[0 simT],simopt,[TU U]);

YT = YTtmp(:,1:4);
LT = YTtmp(:,5);

CLG{1} = [2];
CLG{2} = [1 3];
CLG{3} = [2,4];
CLG{4} = [3];


Guards(1,2).A = [-1 1 0 0];
Guards(1,2).b = 0;

Guards(2,1).A = [1 0 -1 0];
Guards(2,1).b = 0;

Guards(2,3).A = [-1 1 0 0];
Guards(2,3).b = 0;

Guards(3,2).A = [1 0 -1 0];
Guards(3,2).b = 0;

Guards(3,4).A = [-1 1 0 0];
Guards(3,4).b = 0;

Guards(4,3).A = [1 0 -1 0];
Guards(4,3).b = 0;

