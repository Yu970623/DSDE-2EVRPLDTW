function pulish = time_violate(seq_sat,route,data,vrp2e)
    num_dep = vrp2e.num_dep;        %仓库数
    num_sat = vrp2e.num_sat;        %卫星数
    num_cus = vrp2e.num_cus;        %客户数
%     windows = [data.start_time,data.end_time];         %客户时间窗
    windows = data.Windows;
    service = data.Services;         %卫星和客户服务时间
    T1 = data.T1;               %一级路径在时长
    T2 = data.T2;               %二级路径在时长
    CW = data.CW;               %时间窗违反惩罚力度

    % 初始化 time_first 和 time_second
    time_first = zeros(1, length(seq_sat));
    time_second = zeros(1, length(route));
    
    % 定位一级路径的起始点
    loc_dep = find(seq_sat <= num_dep);

    % 计算一级路径访问时刻 time_first
    for i = 1:length(loc_dep)
        start_idx = loc_dep(i);
        if i < length(loc_dep)
            end_idx = loc_dep(i+1) - 1;
        else
            end_idx = length(seq_sat);
        end
        
        for j = start_idx:end_idx
            if j == start_idx
                % 起始点时间为0
                time_first(j) = 0;
            elseif j == start_idx+1
                prev_node = seq_sat(j-1);
                current_node = seq_sat(j);
                time_first(j) = time_first(j-1)+ T1(prev_node, current_node - num_dep);
            else
                prev_node = seq_sat(j-1);
                current_node = seq_sat(j);
                time_first(j) = time_first(j-1) + service(prev_node-num_dep) + T1(prev_node, current_node - num_dep);
            end
        end
    end
    
    % 定位二级路径的起始点
    loc_sat = find(route <= num_sat);

    % 计算二级路径访问时刻 time_second
    for i = 1:length(loc_sat)
        start_idx = loc_sat(i);
        if i < length(loc_sat)
            end_idx = loc_sat(i+1) - 1;
        else
            end_idx = length(route);
        end
        
        for j = start_idx:end_idx
            if j == start_idx
                sat_idx = find(seq_sat == route(j)+num_dep,1); % 找到相应的一级路径序列索引
                time_second(j) = time_first(sat_idx) + service(route(j));
            else
                prev_node = route(j-1);
                current_node = route(j);
                if current_node > num_sat
                    cus_index = current_node - num_sat;
                    normal_time = time_second(j-1) + service(prev_node) + T2(prev_node, current_node - num_sat);
                    if normal_time < windows(cus_index, 1)
                        time_second(j) = windows(cus_index, 1);
                    else
                        time_second(j) = normal_time;
                    end
                end
            end
        end
    end

    punish_time = zeros(num_cus, 1);

    for i = 1:length(route)
        current_node = route(i);
        if current_node > num_sat % 客户节点
            cus_index = current_node - num_sat;
            arrival_time = time_second(i);
            earliest_time = windows(cus_index, 1);
            latest_time = windows(cus_index, 2);
            
            if arrival_time < earliest_time
                punish_time(cus_index,1) = earliest_time - arrival_time;
            elseif arrival_time > latest_time
                punish_time(cus_index,2) = arrival_time - latest_time;
            end
        end
    end
    pulish = sum(punish_time .* CW, 'all');
end
