function [obj1,obj2,c1,c2,c3,c4,c5,All_routes] = aimFcn10(x,data)  
% A strategy-based genotype-to-route mapping for 2E-VRPLD with drones
% 说明：
%   - x 为 [0,1] 的实数编码，长度固定为 D（例如 15），不再与客户数量 num_cus 绑定
%   - x 的前若干维作为“构造策略权重”，用于决定各种特征（时间窗、距离、需求）的重要性
%   - 路径构造仍然调用原有的 instert_cus / insert_sat / msecond / mfirst / time_violate
%   - 同一组 (x,data) 在无随机数影响下会给出确定的 All_routes

    %% --- 0. 确保 x 为行向量、double ---
    if ~isrow(x)
        x = x(:)';
    end
    x = double(x);

    %% --- 1. 基本规模信息与静态数据 ---
    num_cus = data.num_cus;
    num_sat = data.num_sat;
    % num_dep is needed to map satellite nodes in first-echelon route (depot nodes 1..num_dep).
    vrp2e = data.vrp2e;
    if isfield(vrp2e,'num_dep') && ~isempty(vrp2e.num_dep)
        num_dep = vrp2e.num_dep;
    elseif isfield(vrp2e,'rad_ds') && ~isempty(vrp2e.rad_ds)
        num_dep = size(vrp2e.rad_ds,1);
    else
        num_dep = 1;
    end


    % 客户时间窗 (num_cus x 2)
    windows = data.Windows;
    % 顾客需求量：在 extractdata 中对 vrp2e.demand 做了 [zeros(1,num_sat), demand_cus]，
    % data.demand 与其一致，因此顾客部分为 num_sat+1 : num_sat+num_cus
    demand_all = data.demand(:);
    demand_cus = demand_all(num_sat+1:end);   % num_cus x 1

    % 卫星-客户距离矩阵：data.dis_second 的列已对应“客户”，行包含卫星和客户
    % 这里只需要 1:num_sat 行（卫星）到 1:num_cus 列（客户）的距离。
    dis_sc_full = data.dis_second;           % (num_sat+num_cus) x num_cus
    dis_sat_cus = dis_sc_full(1:num_sat,1:num_cus);   % num_sat x num_cus

    %% =====================================================================
    % 2. 构造 “客户静态特征” + 用 x 的前几维做 softmax 得到权重
    %    base_score(i) 越小，客户 i 的全局优先级越高
    %% =====================================================================

    % 时间窗中心、宽度（越早越好 / 越宽越好或越窄越好可以通过权重体现）
    tw_center = mean(windows,2);                 % num_cus x 1
    tw_width  = windows(:,2) - windows(:,1);     % num_cus x 1

    % 每个客户到最近卫星的距离
    min_dis   = min(dis_sat_cus,[],1)';          % num_cus x 1

    % 顾客需求
    dem_cus   = demand_cus(:);                   % num_cus x 1

    % 简单归一化到 [0,1]，避免量纲影响
    norm1 = @(v) ( (v - min(v)) ./ (max(v) - min(v) + eps) );

    f1 = norm1(tw_center);   % 时间窗中心（早晚）
    f2 = tw_width./max(tw_width);    % 时间窗宽度
    f3 = norm1(min_dis);     % 最近卫星距离
    f4 = dem_cus./max(dem_cus);     % 客户需求量

    % 用 x 的前 4 维作为这些特征的权重（softmax 保证非负和为 1）
    K = 4;
    theta = 0.25*ones(K,1);     % 若 x 维度不足，默认均匀
    for k = 1:K
        if k <= numel(x)
            theta(k) = x(k);
        end
    end
    w = exp(theta);
    w = w ./ sum(w);

    % 每个客户的基础得分（越小越优先）
    base_score = w(1)*f1 + w(2)*f2 + w(3)*f3 + w(4)*f4;   % num_cus x 1

    % 额外用 x 的后几维做一个确定性的“小扰动”，避免得分过于规则
    if numel(x) >= 6
        alpha = 0.1;   % 扰动幅度
        idx_cus = (1:num_cus)';   % 1..num_cus
        % 利用 sin 函数构造一个与 x 相关、但平滑的扰动项
        dither = alpha * sin(2*pi*( idx_cus/num_cus * x(5) + x(6) ));
        base_score = base_score + dither;
    end

    %% =====================================================================
    % 3. 基于 base_score + instert_cus 构造二级无人机路径 All_routes2
    %    - 新开一条无人机路径时，从剩余客户中选 base_score 最小的那个
    %    - 在已有路径上插入客户时，按 base_score 排序依次尝试前若干个
    %      具体插入位置与约束判断交由 instert_cus 完成
    %% =====================================================================

    list_served    = [];                  % 已服务客户列表（存客户编号 1..num_cus）
    list_hold      = 1:num_cus;           % 未服务客户列表
    need_newroute2 = true;                % 是否需要新开一条子路径
    All_routes2    = [];                  % 二级路径总串（卫星 + 客户）
    num_drones     = 0;                   %#ok<NASGU> % 当前使用的无人机台数（未直接用）

    while ~isempty(list_hold)
        if need_newroute2
            % --- 3.1 新开一条无人机路径 ---
            % 在剩余客户中选择 base_score 最小的客户
            [~, idx_min] = min(base_score(list_hold));
            new_cus = list_hold(idx_min);    % 客户编号（1..num_cus）

            % 为该客户选择最近的卫星作为起点
            [~, near_sat] = min(dis_sat_cus(:,new_cus));

            % 初始化该子路径： [卫星, 客户节点(加 num_sat 偏移)]
            list_served = [list_served,new_cus];
            list_hold(list_hold == new_cus) = [];
            temp_route = [near_sat, new_cus + num_sat];

            need_newroute2 = false;
            num_drones     = num_drones + 1;

        else
            % --- 3.2 继续在当前子路径中插入新的客户 ---
            if ~isempty(list_hold)
                % 对剩余客户按 base_score 从小到大排序
                temp = list_hold;
                [~, idx_sorted] = sort(base_score(temp),'ascend');
                Priori_cus = temp(idx_sorted);

                % 只尝试前 max_try 个优先客户，以控制 instert_cus 调用次数
                max_try = min(20, numel(Priori_cus));

                for k = 1:max_try
                    temp_cus = Priori_cus(k);              % 候选客户编号
                    c_node   = temp_cus + num_sat;         % 全局节点编号（卫星+客户）
                    % 调用已有插入函数：给出最佳插入位置或者建议开新路
                    % Align with msecond: set the truck-arrival-ready time at this satellite for the current construction state.
                    data_loc = data;
                    try
                        route_tmp = [All_routes2, temp_route];
                        [seq_sat_tmp, ~] = insert_sat(data_loc.vrp2e, route_tmp);
                        time_first_tmp = compute_time_first_local(seq_sat_tmp, data_loc, data_loc.vrp2e);
                        sat_node = num_dep + temp_route(1); % satellite node in seq_sat numbering
                        pos_sat = find(seq_sat_tmp == sat_node, 1, 'first');
                        if ~isempty(pos_sat)
                            data_loc.sat_ready_current = time_first_tmp(pos_sat) + data_loc.Services(sat);
                        else
                            data_loc.sat_ready_current = 0;
                        end
                    catch
                        data_loc.sat_ready_current = 0;
                    end

                    [temp_route1,~,~,need_newroute2] = instert_cus(data_loc,data_loc.vrp2e,temp_route,c_node);

                    % 无法插入当前客户且需要新路：
                    if isequal(temp_route1,temp_route) && need_newroute2
                        % 如果已尝试到当前批次的最后一个候选客户仍无法插入，
                        % 则认为当前子路径已“饱和”，将其并入 All_routes2，下一轮重新开路。
                        if k == max_try
                            All_routes2 = [All_routes2, temp_route];
                            break;
                        else
                            % 否则继续尝试下一位候选客户
                            continue;
                        end
                    else
                        % 成功插入当前客户到 temp_route1
                        temp_route   = temp_route1;
                        list_served  = [list_served,temp_cus];
                        list_hold(list_hold == temp_cus) = [];
                        break;   % 插入一个客户后，重新进入 while 循环
                    end
                end
            end
        end

        % 若所有客户均已服务，则把当前子路径并入 All_routes2
        if isempty(list_hold)
            All_routes2 = [All_routes2, temp_route];
        end
    end

    %% ============================================
    % 4. 调用原有的一、二级评价函数，计算目标函数与约束违反
    %% =====================================================================

    % 一级卡车路径构造
    [seq_sat,cap_sat] = insert_sat(data.vrp2e,All_routes2);

    % 二级无人机成本与约束（已修改为：每条无人机回路从最后一个客户返回到最近的卫星）
    [equip_cost2,energy_cost2,c_weight,c_energy] = msecond(data.vrp2e,All_routes2, seq_sat, data); 
    % 计算每条无人机回路实际返回的卫星（最近卫星），用于结果可视化与分析
    vrp2e   = data.vrp2e;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;
    dis_sc  = vrp2e.dis_sc;
    route   = All_routes2;
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
    
    % All_routes 结构：
    %   {1} seq_sat : 一级卡车路径（仓库-卫星）
    %   {2} All_routes2 : 二级无人机路径（仅含“起始卫星 + 客户序列”，不含返回卫星）
    %   {3} end_sat : 每条无人机回路“最终返回的卫星”索引（与 All_routes2 中的各条子路径一一对应）
    All_routes = {seq_sat;All_routes2;end_sat};
    
    % 一级成本
    [equip_cost1,energy_cost1,fvlt1] = mfirst(data.vrp2e,seq_sat,cap_sat);
    
    % 目标与约束
    obj1 = equip_cost1 + equip_cost2;     % 第一、二级设备成本之和
    obj2 = energy_cost1 + energy_cost2;   % 第一、二级能耗成本之和
    c1   = c_weight;                      % 无人机载重约束违反
    c2   = c_energy;                      % 无人机续航约束违反
    c3   = fvlt1;                         % 卡车载重 / 数量约束违反
    c4   = time_violate(seq_sat,All_routes2,data,data.vrp2e); % 时间窗违反（仍按照原有表示）
    c5   = c1 + c2 + c3 + c4;             % 总约束违反
end