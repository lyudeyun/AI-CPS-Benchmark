function [T XT YT LT CLG Guards] = sldemo_autotrans_bb_04(X0,simT,TU,U)

Guards = [];

simopt = simget('autotrans_mod04');
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YTtmp] = sim('autotrans_mod04',[0 simT],simopt,[TU U]);

YT = YTtmp(:,1:2);
LT = YTtmp(:,3);
%plot(T,LT)

    CLG{1} = 2;
    CLG{2} = [1,3];
    CLG{3} = [2,4];
    CLG{4} = 3;

end
