[vrp2e,timewindows,timeservice,demand,type,dis_ds,dis_sc,num_dep,num_sat,num_cus] = extractdata(1,5);
data.vrp2e = vrp2e;
data.numN=num_dep+num_sat+num_cus;  %节点数量
data.num_dep = num_dep;             %仓库数量
data.num_sat = num_sat;             %卫星数量
data.num_cus = num_cus;             %客户数量
data.demand=demand;                 %客户需求量
data.type=type;                     %客户需求类型
data.Windows=timewindows;           %这个可能需要松弛
data.Services=timeservice;          %客户服务时间
data.dis_first = dis_ds(:,num_dep+1:end);   %一级路径矩阵
data.dis_second = dis_sc(:,num_sat+1:end);  %二级路径矩阵
data.v = [30,10];                   %一、二级路径的车辆速度
data.T1 = 60*data.dis_first/data.v(1); %一级路径的路段运行时长
data.T2 = 60*data.dis_second/data.v(2);%二级路径的路段运行时长
data.CW=[30,50];                    %时间窗惩罚系数

data.start_time = data.Windows(:,1);
data.end_time = data.Windows(:,2);

data.maxE=550;  %最大电池容量
data.maxW=5;    %最大载重重量
%[2,6,5,2,10,7,12,2,16,18,14,9,2,8,4,15,17,13,11]
%[2,11,2,8,2,9,2,14,2,16,2,18,2,4,15,2,10,7,12,2,6,17,5,13]

%[3,13,3,20,3,7,12,3,16,17,3,10,8,9,11,3,6,18,15,19,14] 
%[3,18,17,3,15,16,3,7,8,9,3,6,19,14,13,3,12,10,11,20]

[equip_cost2, energy_cost2, c_weight, c_energy] = msecond(vrp2e,[3,18,17,3,15,16,3,7,8,9,3,6,19,14,13,3,12,10,11,20],[2,6], data)

[seq_sat,cap_sat] = insert_sat(vrp2e,[3,18,17,3,15,16,3,7,8,9,3,6,19,14,13,3,12,10,11,20]);

[equip_cost1,energy_cost1,fvlt1] = mfirst(vrp2e,[2,6],[0,12])

f1 = equip_cost1+equip_cost2
f2 = energy_cost1+energy_cost2

A = f1
B = f2