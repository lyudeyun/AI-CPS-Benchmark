nT=[];
nR=[];
numOfFals=0;
if table_id==1
    runtimes=[];
    for iii=1:outer_runs
        if results.run(iii).falsified==1
            nT=[nT; results.run(iii).nTests];
            numOfFals=numOfFals+1;
        else
            nR=[nR; results.run(iii).bestCost ];
        end
        runtimes=[runtimes;results.run(iii).time];
    end
else
    
    for iii=1:outer_runs
        if results_suff(iii).run.falsified==1
            nT=[nT; results_suff(iii).run.nTests];
            numOfFals=numOfFals+1;
        else
            nR=[nR; results_suff(iii).run.bestCost ];
        end
        
    end
end
disp('**************************************************************')
disp([' Reporting the Results of Table : ',num2str(table_id),' , Row ',num2str(opt_id)])
disp('**************************************************************')
disp([' Number of falsifications : ',num2str(numOfFals),'/',num2str(outer_runs)])
disp([' Minimum Number of Tests : ',num2str(min(nT))])
disp([' Maximum Number of Tests : ',num2str(max(nT))])
disp([' Average Number of Tests : ',num2str(round(mean(nT)))])
disp('**************************************************************')
disp(' The Robustness Results for the unfalsified experiments:')
disp([' Minimum Robustness : ',num2str(min(nR))])
disp([' Maximum Robustness : ',num2str(max(nR))])
disp([' Average Robustness : ',num2str(mean(nR))])
disp('**************************************************************')
disp(' The Running Time of each Falsification Attempt:')
disp([' Minimum Runtime : ',num2str(min(runtimes))])
disp([' Maximum Runtime : ',num2str(max(runtimes))])
disp([' Average Runtime : ',num2str(mean(runtimes))])
disp('**************************************************************')

