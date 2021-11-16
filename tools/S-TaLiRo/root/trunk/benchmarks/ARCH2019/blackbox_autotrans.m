function [T, XT, YT, LT, CLG, Guards] = blackbox_autotrans(~,simT,TU,U)

Guards = [];

simopt = simget('Autotrans_shift');
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YTtmp] = sim('Autotrans_shift',[0 simT],simopt,[TU U]);

YT = YTtmp(:,1:2);
LT = YTtmp(:,3);
%plot(T,LT)

    CLG{1} = 2;
    CLG{2} = [1,3];
    CLG{3} = [2,4];
    CLG{4} = 3;

end
