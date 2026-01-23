function [obj1,obj2,c1,c2,c3,c4,c5,All_routes] = aimFcn10(x,data)  
% A strategy-based genotype-to-route mapping for 2E-VRPLD with drones


    if ~isrow(x)
        x = x(:)';
    end
    x = double(x);

    num_cus = data.num_cus;
    num_sat = data.num_sat;

    % 客户时间窗 (num_cus x 2)
    windows = data.Windows;
    demand_all = data.demand(:);
    demand_cus = demand_all(num_sat+1:end);   % num_cus x 1

    dis_sc_full = data.dis_second;                  % (num_dep+num_sat+num_cus) x num_cus
    dis_sat_cus = dis_sc_full(1:num_sat,1:num_cus); % num_sat x num_cus


    T2 = data.T2;


    tw_center = mean(windows,2);                 % num_cus x 1
    tw_width  = windows(:,2) - windows(:,1);     % num_cus x 1


    [min_dis, near_sat] = min(dis_sat_cus,[],1); % 1 x num_cus
    min_dis   = min_dis(:);                      % num_cus x 1
    near_sat  = near_sat(:);                     % num_cus x 1


    dem_cus   = demand_cus(:);                   % num_cus x 1

    idx_cus = (1:num_cus)';
    travel_time = zeros(num_cus,1);
    for i = 1:num_cus
        s = near_sat(i);             % 最近卫星
        travel_time(i) = T2(s,i);    % 从该卫星直飞到客户 i 的时间（分钟）
    end
    slack = windows(:,2) - (windows(:,1) + travel_time);   % slack 可能为负，表示“理论上也较难满足”


    load_sens = (5 + dem_cus).^(3/2) .* min_dis;

    norm1 = @(v) ( (v - min(v)) ./ (max(v) - min(v) + eps) );

    f1 = norm1(tw_center);   % 时间窗中心（早晚）
    f2 = tw_width./max(tw_width);    % 时间窗宽度
    f3 = norm1(min_dis);     % 最近卫星距离
    f4 = dem_cus./max(dem_cus);     % 客户需求量
    f5 = norm1(slack);       % 时间松弛度（越小越紧迫）
    f6 = norm1(load_sens);   % 载重-能耗敏感度近似

    % 用 x 的前 6 维作为这些特征的权重（softmax 保证非负且和为 1）
    K = 6;
    theta = ones(K,1)/K;     % 若 x 维度不足，默认均匀
    for k = 1:K
        if k <= numel(x)
            theta(k) = x(k);
        end
    end
    w = exp(theta);
    w = w ./ sum(w);

    % 每个客户的基础得分（越小越优先）
    base_score = w(1)*f1 + w(2)*f2 + w(3)*f3 + w(4)*f4 + w(5)*f5 + w(6)*f6;   % num_cus x 1


    if numel(x) >= 8
        alpha = 0.1;   % 扰动幅度
        phi1  = x(7);
        phi2  = x(8);
    else
        alpha = 0.1;
        phi1  = 0.37;
        phi2  = 0.61;
    end
    dither = alpha * sin(2*pi*( idx_cus/num_cus * phi1 + phi2 ));
    base_score = base_score + dither;


    list_hold   = 1:num_cus;           % 未服务客户（1..num_cus）
    routes2_cell = {};                % 每个元素是一条子路段 [sat, cus, cus, ...]
    used_sats   = [];                 % 当前已启用的卫星集合


    MAX_TRY_PER_STEP = 20;

    while ~isempty(list_hold)
        % 选一个优先客户（base_score 越小越优先）
        temp = list_hold;
        [~, idx_sorted] = sort(base_score(temp), 'ascend');
        Priori_cus = temp(idx_sorted);
        max_try = min(MAX_TRY_PER_STEP, numel(Priori_cus));

        inserted_any = false;
        for kk = 1:max_try
            cus_id = Priori_cus(kk);
            c_node = cus_id + num_sat;

            % 传入一些“全局信息”给 instert_cus（不改变接口，只是 data 增加字段）
            data_local = data;
            data_local.used_sats = used_sats;
            data_local.remain_customers = list_hold;  % 仍未服务客户（用于卫星“中心性”偏好）

            best_mode = 0;  % 1=插入已有子路段, 2=新开子路段
            best_cost = inf;
            best_ridx = 0;
            best_route = [];

            % 3.2 尝试插入所有已有子路段（全局插入，避免早期决策锁死）
            for r = 1:numel(routes2_cell)
                [cand_route, inc_cost, ~, need_new] = instert_cus(data_local, data_local.vrp2e, routes2_cell{r}, c_node);
                if need_new
                    continue;
                end
                if inc_cost < best_cost
                    best_cost  = inc_cost;
                    best_mode  = 1;
                    best_ridx  = r;
                    best_route = cand_route;
                end
            end

            [new_route, new_cost, ~, ~] = instert_cus(data_local, data_local.vrp2e, [], c_node);
            if new_cost < best_cost
                best_cost  = new_cost;
                best_mode  = 2;
                best_route = new_route;
            end

            if best_mode == 1
                routes2_cell{best_ridx} = best_route;
                inserted_any = true;
            elseif best_mode == 2
                routes2_cell{end+1} = best_route;
                used_sats = unique([used_sats, best_route(1)]);
                inserted_any = true;
            end

            if inserted_any
                list_hold(list_hold == cus_id) = [];
                break; % 每次 while 至少插入一个客户
            end
        end

        if ~inserted_any
            cus_id = list_hold(1);
            c_node = cus_id + num_sat;
            data_local = data;
            data_local.used_sats = used_sats;
            data_local.remain_customers = list_hold;
            [new_route, ~, ~, ~] = instert_cus(data_local, data_local.vrp2e, [], c_node);
            routes2_cell{end+1} = new_route;
            used_sats = unique([used_sats, new_route(1)]);
            list_hold(list_hold == cus_id) = [];
        end
    end

if numel(routes2_cell) >= 2
    MAX_REMOVE_TRY = 3; % 控制尝试次数，避免过重
    remove_cnt = 0;
    
    while numel(routes2_cell) >= 2 && remove_cnt < MAX_REMOVE_TRY
        % 按子路段客户数从小到大尝试移除
        lens = zeros(1, numel(routes2_cell));
        for r = 1:numel(routes2_cell)
            lens(r) = sum(routes2_cell{r} > num_sat);
        end
        [~, ridx_order] = sort(lens, 'ascend');
        
        removed_any = false;
        for kk = 1:numel(ridx_order)
            rr = ridx_order(kk);
            if rr < 1 || rr > numel(routes2_cell)
                continue;
            end
            route_r = routes2_cell{rr};
            cus_r = route_r(route_r > num_sat);
            if isempty(cus_r)
                continue;
            end
            
            % 备份
            routes_backup = routes2_cell;
            ok_move_all = true;
            
            % 临时移除该路段
            routes2_cell(rr) = [];
            if isempty(routes2_cell)
                used_sats_tmp = [];
            else
                used_sats_tmp = unique(cellfun(@(x) x(1), routes2_cell));
            end
            
            % 逐个客户迁移
            for cc = 1:numel(cus_r)
                c_node = cus_r(cc);
                data_local = data;
                data_local.used_sats = used_sats_tmp;
                data_local.remain_customers = []; % 不需要
                
                best_cost = inf;
                best_ridx = 0;
                best_route = [];
                
                for r = 1:numel(routes2_cell)
                    [cand_route, inc_cost, ~, need_new] = instert_cus(data_local, data_local.vrp2e, routes2_cell{r}, c_node);
                    if need_new
                        continue;
                    end
                    if inc_cost < best_cost
                        best_cost = inc_cost;
                        best_ridx = r;
                        best_route = cand_route;
                    end
                end
                
                if best_ridx == 0
                    ok_move_all = false;
                    break;
                else
                    routes2_cell{best_ridx} = best_route;
                    if isempty(routes2_cell)
                        used_sats_tmp = [];
                    else
                        used_sats_tmp = unique(cellfun(@(x) x(1), routes2_cell));
                    end
                end
            end
            
            if ok_move_all
                remove_cnt = remove_cnt + 1;
                removed_any = true;
                % 更新 used_sats
                if isempty(routes2_cell)
                    used_sats = [];
                else
                    used_sats = unique(cellfun(@(x) x(1), routes2_cell));
                end
                break; % 成功移除一条后，重新排序/重启 while
            else
                % 回滚
                routes2_cell = routes_backup;
            end
        end
        
        if ~removed_any
            break;
        end
    end
end

vrp2e   = data.vrp2e;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    % 邻域搜索强度控制：x(9) 越大，允许的局部改进次数越多（最多 2 次）
    if numel(x) >= 9
        ls_level = x(9);
    else
        ls_level = 0.5;  % 默认中等强度
    end
    if ls_level < 1/3
        num_LS_iter = 3;
    elseif ls_level < 2/3
        num_LS_iter = 5;
    else
        num_LS_iter = 10;
    end

    % 先对构造出的 All_routes2 做一次完整评价，作为当前“基准解”
    if isempty(All_routes2)
        % 极端情况：没有二级路径
        equip_cost2 = 0; energy_cost2 = 0; c_weight = 0; c_energy = 0;
        seq_sat     = [];
        cap_sat     = [];
        equip_cost1 = 0; energy_cost1 = 0; fvlt1 = 0;
        c4          = 0;
        obj1        = 0; obj2 = 0;
        c1 = 0; c2 = 0; c3 = 0; c5 = 0;
        All_routes = {seq_sat;All_routes2;[]};
        return;
    end

    [seq_sat,cap_sat] = insert_sat(vrp2e,All_routes2);
    % 二级无人机成本与约束（已修改为：每条无人机回路从最后一个客户返回到最近的卫星）
    [equip_cost2,energy_cost2,c_weight,c_energy] = msecond(data.vrp2e,All_routes2, seq_sat, data); 
    [equip_cost1,energy_cost1,fvlt1] = mfirst(vrp2e,seq_sat,cap_sat);
    c4 = time_violate(seq_sat,All_routes2,data,vrp2e);

    obj1 = equip_cost1 + equip_cost2;
    obj2 = energy_cost1 + energy_cost2;
    c1   = c_weight;
    c2   = c_energy;
    c3   = fvlt1;
    c5   = c1 + c2 + c3 + c4;

    % 用一个简单的罚函数聚合目标，用于邻域比较（仅用于局部搜索内部，不影响外部多目标优化）
    bigM   = 1e6;
    F_best = (obj1 + obj2) + bigM * c5;

    % 记录当前最优解
    routes2_best   = All_routes2;
    seq_sat_best   = seq_sat;
    cap_sat_best   = cap_sat;
    obj1_best      = obj1;
    obj2_best      = obj2;
    c1_best        = c1;
    c2_best        = c2;
    c3_best        = c3;
    c4_best        = c4;
    c5_best        = c5;

    % --- 邻域搜索：同一路内 1-0 relocate（移除一个客户，再插入到同一路的其他位置） ---
    for iter = 1:num_LS_iter
        improvement = false;
        route_cur   = routes2_best;
        loc_sat_cur = find(route_cur <= num_sat);

        for r_idx = 1:numel(loc_sat_cur)
            start_idx = loc_sat_cur(r_idx);
            if r_idx < numel(loc_sat_cur)
                end_idx = loc_sat_cur(r_idx+1) - 1;
            else
                end_idx = numel(route_cur);
            end

            % 该子路内的客户位置（排除卫星本身）
            pos_vec = (start_idx+1):end_idx;
            pos_vec = pos_vec(route_cur(pos_vec) > num_sat);  % 只考虑客户节点

            for p_id = 1:numel(pos_vec)
                i = pos_vec(p_id);          % 要搬移的客户位置
                node = route_cur(i);        % 该客户节点编号

                % 尝试将该客户插入到同一路内其它位置
                for new_pos = (start_idx+1):end_idx
                    if new_pos == i
                        continue;
                    end

                    % 构造候选路径：先删除，再插入
                    cand_route = route_cur;
                    cand_route(i) = [];      % 删除原位置 i

                    if new_pos < i
                        insert_idx = new_pos;
                    else
                        insert_idx = new_pos - 1;  % 删除后索引左移一位
                    end
                    cand_route = [cand_route(1:insert_idx-1), node, cand_route(insert_idx:end)];

                    % 评价候选解（完整的一、二级成本 + 约束）
                    [seq_sat_c,cap_sat_c] = insert_sat(vrp2e,cand_route);
                    [equip_cost2_c,energy_cost2_c,c_weight_c,c_energy_c] = msecond(vrp2e,cand_route, seq_sat_c, data);
                    [equip_cost1_c,energy_cost1_c,fvlt1_c] = mfirst(vrp2e,seq_sat_c,cap_sat_c);
                    c4_c = time_violate(seq_sat_c,cand_route,data,vrp2e);

                    obj1_c = equip_cost1_c + equip_cost2_c;
                    obj2_c = energy_cost1_c + energy_cost2_c;
                    c1_c   = c_weight_c;
                    c2_c   = c_energy_c;
                    c3_c   = fvlt1_c;
                    c5_c   = c1_c + c2_c + c3_c + c4_c;

                    F_c = (obj1_c + obj2_c) + bigM * c5_c;  % (kept) penalty value (not used for acceptance)

                    better = (c5_c < c5_best - 1e-9) || (abs(c5_c - c5_best) <= 1e-9 && (obj1_c + obj2_c) < (obj1_best + obj2_best) - 1e-9);

                    % 若候选解在“罚函数意义下”更优，则接受第一处改进并重新开始下一轮邻域搜索
                    if better
                        F_best        = F_c;
                        routes2_best  = cand_route;
                        seq_sat_best  = seq_sat_c;
                        cap_sat_best  = cap_sat_c;
                        obj1_best     = obj1_c;
                        obj2_best     = obj2_c;
                        c1_best       = c1_c;
                        c2_best       = c2_c;
                        c3_best       = c3_c;
                        c4_best       = c4_c;
                        c5_best       = c5_c;
                        improvement   = true;
                        break;   % 跳出 new_pos 循环
                    end
                end

                if improvement
                    break;       % 跳出 p_id 循环
                end
            end

            if improvement
                break;           % 跳出 r_idx 循环，进入下一轮 iter
            end
        end

        if ~improvement
            break;               % 若本轮未发现改进，则提前结束邻域搜索
        end
    end

    % 使用邻域搜索后的最优二级路径及对应的一、二级结果
    All_routes2 = routes2_best;
    seq_sat     = seq_sat_best;
    cap_sat     = cap_sat_best;
    obj1        = obj1_best;
    obj2        = obj2_best;
    c1          = c1_best;
    c2          = c2_best;
    c3          = c3_best;
    c4          = c4_best;
    c5          = c5_best;

   dis_sc = vrp2e.dis_sc;
    route  = All_routes2;
    end_sat = [];
    if ~isempty(route)
        loc_sat = find(route <= num_sat);
        nRoute  = numel(loc_sat);
        end_sat = zeros(1,nRoute);
        for r = 1:nRoute
            start_idx = loc_sat(r);
            if r < nRoute
                end_idx = loc_sat(r+1) - 1;
            else
                end_idx = numel(route);
            end
            last_node = route(end_idx);
            if last_node > num_sat
                % 根据“最后一个客户节点”到各卫星的距离，选择最近卫星
                dists        = dis_sc(1:num_sat,last_node);
                [~,best_s]   = min(dists);
                end_sat(r)   = best_s;
            else
                % 极端情况：该路线没有客户，仅含卫星，则视为返回原卫星
                end_sat(r) = route(start_idx);
            end
        end
    end
    All_routes = {seq_sat;All_routes2;end_sat};
end
