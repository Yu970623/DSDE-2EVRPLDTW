results =  xlsread('F:\Onedrive\Experiment\2EVRPLDTW\Data\6415_HV.xlsx','Sheet1');
Names = {'CMOEAD','PPS','DN-NSGAII','CMMO','SFEA','DSDE'};
boxplot(results,Names)
% xlabel('HV Metric');
ylabel('HV Metric');
% ylim([0.02,0.04]);




