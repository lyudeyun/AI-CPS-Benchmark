function [T XT YT LT CLG Guards] = sldemo_autotrans_bb_02(X0,simT,TU,U)

Guards = [];

simopt = simget('sldemo_autotrans_mod02');
simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
[T, XT, YTtmp] = sim('sldemo_autotrans_mod02',[0 simT],simopt,[TU U]);

YT = YTtmp(:,1:2);
LT = YTtmp(:,3);

for ii = 1:4
    CLG{ii} = [4+ii 8+ii];
    if ii==1
        CLG{ii+4} = ii;
    else
        CLG{ii+4} = [ii ii-1];
    end
    if ii==4
        CLG{ii+8} = ii;
    else
        CLG{ii+8} = [ii ii+1];
    end
end

end
