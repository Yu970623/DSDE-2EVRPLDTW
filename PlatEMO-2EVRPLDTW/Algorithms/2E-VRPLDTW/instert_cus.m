function [seq_r,min_cost,Remain_battery,need_newroute2] = instert_cus(data,vrp2e,route,c)
    fleet = vrp2e.fleet;  %获取车队信息，如每辆车的最大容量等。
    demand = vrp2e.demand; %获取客户的需求量
    dis_ds = vrp2e.dis_ds; 
    dis_sc = vrp2e.dis_sc;  %卫星与客户之间的距离 
    num_dep = vrp2e.num_dep;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    cus_type = vrp2e.type;
    E = vrp2e.E;  %电池总功率
    V = vrp2e.V;    %无人机飞行速度

    penalty = 1000;  %设置一个惩罚值，用于约束条件的违规情况
    %----计算当前路径中已分配的车辆数量和其累积装载量----
    num_fleet     = sum(route<=num_sat);
    cap_s         = zeros(1,2*num_cus);                  %初始化路段实时重量
    nums_c        = num_sat+num_cus;
    len_route     = numel(route);
    seq_r         = zeros(1,2*num_cus); %卫星-客户对应关系记录
    seq_r(1:len_route) = route;
    
    %----贪婪插入----
    exist_sat = route(route<=num_sat);
    [dis_dep,far_dep] = max(dis_ds(1:num_dep,exist_sat+num_dep));
    cost_newr        = 2*dis_sc(c,exist_sat);   %当前客户分别与所有卫星的距离
    [min_dis,~]         = min(cost_newr);       % 找出当前客户最近的卫星和对应的距离

    route      = seq_r(1:len_route);        %已有路径
    sign_fleet = cumsum(route<=num_sat,2);  %卫星和客户对应的无人机（按行求累积和）
    loc_sat    = find(route<=num_sat);      %卫星在路径中的位置

    % 将卫星和车辆的相应路径记录
    rback      = route; 
    rback(loc_sat(2:end)) = route(loc_sat(1:end-1));
    route2     = [rback(2:end),route(loc_sat(end))];    %返回卫星的路径
    cap_sc = cap_realtime(vrp2e,[route,0]);
    power = (5+cap_sc).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

    %计算当前已有路径的总能耗
    Energy_route = power.* dis_sc(route+(route2-1)*nums_c)./V;  %已有路径各路段能耗
    Energy = [arrayfun(@(i)[sum(Energy_route(loc_sat(i):loc_sat(i+1)-1))], 1:length(loc_sat)-1, 'UniformOutput', false),sum(Energy_route(loc_sat(end):end))];  %计算原路径的总能耗
    Energy = arrayfun(@(x) Energy(x), sign_fleet);
    Energy = cell2mat(Energy);
    %根据取-送情况分别计算插入新增用户后可能的代价, 无人机载重+续航能力+时间窗和数量是否超过约束
    if strcmp(cus_type(c-num_sat), 'pickup')
        %插入到哪个位置，该位置之后的cap_sc+demand(c),之前的不加
        W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1), demand(c) + cap_sc(n-1), cap_sc(n:end)+demand(c)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
        W_all = [];
        cap_p = cap_sc(1:loc_sat(end)-1);
        W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
        W_distribution = [W_all,W2];
        cost = (5).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*min_dis./V;
    else
        %插入到哪个位置，该位置之前的cap_sc+demand(c)，之后的不加
        W_end = arrayfun(@(n) [cap_sc(loc_sat(end):n-1)+demand(c), cap_sc(n-1), cap_sc(n:end)], loc_sat(end)+1:length(cap_sc)+1, 'UniformOutput', false);
        W_all = [];
        cap_p = cap_sc(1:loc_sat(end)-1);
        W2 = cellfun(@(x) [cap_p, x], W_end, 'UniformOutput', false);
        W_distribution = [W_all,W2];
        cost = (5+demand(c)).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4)*min_dis./V;
    end
    Max_W=cellfun(@(x) max(x(1:end)), W_distribution);  %获取所有插入情况的最大值（超重限制阈值）
    route_c = arrayfun(@(pos) [route(1:pos-1), c, route(pos:end)], 2:length(route)+1, 'UniformOutput', false);  %前段路径
    loc_sat_c = cellfun(@(x) find(x <= num_sat), route_c, 'UniformOutput', false);    %添加新客户后的卫星坐标
    rback = cellfun(@(x) x, route_c, 'UniformOutput', false);
    for i = 1:length(route_c)% 更新 rback 中的每个 cell
        rback{i}(loc_sat_c{i}(2:end)) = route_c{i}(loc_sat_c{i}(1:end-1));
        route_c2{i} = [rback{i}(2:end),route_c{i}(loc_sat_c{i}(end))];
    end% 
    dis_route = cellfun(@(x, y) dis_sc(x + (y - 1) * nums_c), route_c, route_c2, 'UniformOutput', false);%完整路径对应的距离长度
    power_route = cellfun(@(W) (5+W).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4), W_distribution, 'UniformOutput', false);   %每个路段的功率
    segroute_time = cellfun(@(d) 60.*d./V, dis_route, 'UniformOutput', false);      %路段各自的飞行时间/分钟
    arrive_time = cellfun(@(x) cumsum([x(1)+60*dis_dep/30+10, x(2:end)+10]) + [0, x(1:end-1)+10], segroute_time, 'UniformOutput', false);   %基于最远卫星情况下的实时抵达时刻
    
    processed = cellfun(@(x) x(2:end) - num_sat, route_c, 'UniformOutput', false);          % 去掉每个数组的第一个元素，并减去 num_sat
    early_window = cellfun(@(idx) data.Windows(idx, 1).', processed, 'UniformOutput', false); % 提取 data.Windows 的第一列到 early_window
    last_window = cellfun(@(idx) data.Windows(idx, 2).', processed, 'UniformOutput', false);  % 提取 data.Windows 的第二列到 last_window
    time_gap = cell(size(arrive_time));
    for i = 1:length(arrive_time)
        a = arrive_time{i};
        e = early_window{i};
        n = length(e);
        gap = zeros(1, n);
        
        for j = 1:n
            gap(j) = e(j) - a(j);
            if gap(j) > 0
                a(j:end) = a(j:end) + gap(j);
            end
        end
        
        time_gap{i} = gap;
        arrive_time{i} = a;  % 更新调整后的 arrive_time
    end

    Energy_distribution = cellfun(@(p, d, tg) [p(1:end-1) .* (d(1:end-1)/V + (tg > 0) .* tg/60), ...  % 前 n 位
    p(end) * d(end)/V], power_route, dis_route, time_gap, 'UniformOutput', false);
    %插入不同位置的不同总功耗
    time_vio = zeros(1,length(route_c));
    for i =1:length(route_c)
        time_vio(i) = time_violate([far_dep,exist_sat+num_dep],route_c{i},data,vrp2e);
    end
    group = [];
    if length(loc_sat)>1
        for i=1:length(loc_sat)-1
            group{i} = loc_sat(i):loc_sat(i+1)-1;
        end
        group{length(group)+1} = loc_sat(end):length(Energy_distribution);
    else
        group = {1:length(Energy_distribution)};
    end
    
    for i = 1:length(group)
        if i<length(group)
            for j = group{i}
                Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(i):loc_sat_c{j}(i+1)-1));
            end
        else
            for j = group{i}
                Energy_c {j} = sum(Energy_distribution{j}(loc_sat_c{j}(end):end));
            end
        end
    end
    punish = [penalty * (max(Max_W-fleet(2,1),0)+max(cell2mat(Energy_c)-E,0)+max(time_vio,0)),penalty * max(num_fleet+1-fleet(2,2),0)];
    cost_c     = [cell2mat(Energy_c)-Energy,cost*10]+punish;
    [min_cost,loc_in] = min(cost_c);  % 找出最小代价的插入位置
    if loc_in <= len_route  % 如果插入位置在当前路径范围内
        need_newroute2 = false;
        % 在插入位置后移动现有路径中的客户
        seq_r(loc_in+2:len_route+1) = seq_r(loc_in+1:len_route);
        seq_r(loc_in+1) = c;
        len_route       = len_route+1;
        cap_s(1:numel(W_distribution{loc_in})) = W_distribution{loc_in};% 已有车辆的装载量更新
        Remain_battery = (E-Energy_c{loc_in})/E;
    else       % 如果插入位置超出当前路径范围，则代表超过容量或续航限制
        min_cost = Inf;
        need_newroute2 = true;
        Remain_battery = 1;
    end
    seq_r(len_route+1:end) = []; % 删除多余的空白部分
end

