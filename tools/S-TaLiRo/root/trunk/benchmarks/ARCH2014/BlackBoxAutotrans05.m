function [T XT YT LT CLG Guards] = sldemo_autotrans_bb_05(X0,simT,TU,U)

Guards = [];

simopt = simget('autotrans_mod05');
simopt = simset(simopt,'SaveFormat','Array');
%set_param('autotrans_mod05','SimulationMode','accelerator')
%simopt = simset(simopt,'RapidAcceleratorUpToDateCheck','off');
[T, XT, YTtmp] = sim('autotrans_mod05',[0 simT],simopt,[TU U]);
%,'SimulationMode', 'rapid','RapidAcceleratorUpToDateCheck', 'off'
YT = YTtmp(:,1);
LT = YTtmp(:,2);
%plot(T,LT)

    CLG{1} = 2;
    CLG{2} = [1,3];
    CLG{3} = [2,4];
    CLG{4} = 3;

end
