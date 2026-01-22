% repair2.m
% --------------------------第二层路径优化（带时间窗可行性预检，且不丢客户）--------------------------
function seq_r = repair2(vrp2e,seq_cs,seq_c,s_off,sm)
    if sm < 4
        seq_r = greedyin2(vrp2e,seq_cs,seq_c,s_off);
    else
        seq_r = greedyinr2(vrp2e,seq_cs,seq_c,s_off);
    end
end


% ========================================================================
% 贪婪插入（队列式 while，不丢客户；支持多卫星尝试；若长期无法插入则安全退出并保留客户）
% ========================================================================
function seq_r = greedyin2(vrp2e,route,seq_c,s_off)

    fleet   = vrp2e.fleet;
    dis_sc  = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    bigM      = 1e8;
    len_route = numel(route);

    seq_r = zeros(1,2*num_cus + max(10, numel(seq_c))*2);
    if len_route > 0
        seq_r(1:len_route) = route;
    end

    % -------- 队列式处理，保证不会因 for+append 导致客户丢失 --------
    q = seq_c(:)';               % queue
    no_progress_round = 0;       % 连续未插入计数（用来防止死循环）

    while ~isempty(q)

        c = q(1);
        q(1) = [];

        % 给客户挑一个“可用卫星候选序列”（按距离从近到远，过滤 s_off）
        sat_list = rank_satellites(vrp2e, c, s_off);

        if len_route == 0
            % 必须开第一条回路：尽量找一个可行的 [s,c]
            [s_sel, ok] = first_feasible_sat(vrp2e, c, sat_list);
            if ok
                seq_r(1:2) = [s_sel, c];
                len_route = 2;
                no_progress_round = 0;
            else
                % 说明对所有卫星，[s,c] 都不可行（极端/数据不可行）
                % 不能丢客户：把它放回队尾，累计一轮无进展
                q = [q, c];
                no_progress_round = no_progress_round + 1;

                % 如果整轮都无法开路，安全退出：把剩余客户以 [最近卫星, 客户] 形式附到后面（不丢客户）
                if no_progress_round > numel(q) + 1
                    seq_r(len_route+1:len_route+2) = [sat_list(1), c];
                    len_route = len_route + 2;
                    for t = 1:numel(q)
                        sat_list_t = rank_satellites(vrp2e, q(t), s_off);
                        seq_r(len_route+1:len_route+2) = [sat_list_t(1), q(t)];
                        len_route = len_route + 2;
                    end
                    q = [];
                end
            end
            continue;
        end

        route_now = seq_r(1:len_route);
        num_pos   = len_route + 1;
        cost_candidates = inf(1, num_pos);
        best_route_candidates = cell(1, num_pos);

        % --- 1) 尝试插入到现有路径不同位置 ---
        for pos = 2:num_pos
            route_cand = [route_now(1:pos-1), c, route_now(pos:end)];
            if ~route_tw_feasible(vrp2e, route_cand)
                continue;
            end
            [equip2,energy2,c_weight,c_energy] = msecond(vrp2e, route_cand);
            
            if c_weight > 0 || c_energy > 0
                continue;
            end
num_fleet_cand = sum(route_cand <= num_sat);
            c_fleet = max(num_fleet_cand - fleet(2,2),0);
            cost_candidates(pos-1) = equip2 + energy2 + bigM*(c_weight + c_energy + c_fleet);
            best_route_candidates{pos-1} = route_cand;
        end

        % --- 2) 尝试新开一条回路（用最先可行的卫星；若都不可行则该选项保持 INF） ---
        [s_new, ok_new] = first_feasible_sat(vrp2e, c, sat_list);
        if ok_new
            route_new = [route_now, s_new, c];
            % route_new 也必须可行（保险）
            if route_tw_feasible(vrp2e, route_new)
                [equip2,energy2,c_weight,c_energy] = msecond(vrp2e, route_new);
        if c_weight > 0 || c_energy > 0
            % infeasible for capacity/endurance -> do not consider this option
            % (keep cost as INF)
        else
num_fleet_cand = sum(route_new <= num_sat);
                c_fleet = max(num_fleet_cand - fleet(2,2),0);
                cost_candidates(num_pos) = equip2 + energy2 + bigM*(c_weight + c_energy + c_fleet);
                
        end
best_route_candidates{num_pos} = route_new;
            end
        end

        [best_cost, best_idx] = min(cost_candidates);

        if isinf(best_cost)
            % 所有方案都不可行：不能丢客户，推回队尾
            q = [q, c];
            no_progress_round = no_progress_round + 1;

            % 若连续一整轮都无法插入任何客户，安全退出并保留剩余客户（避免死循环）
            if no_progress_round > numel(q) + 1
                sat_list_c = rank_satellites(vrp2e, c, s_off);
                seq_r(len_route+1:len_route+2) = [sat_list_c(1), c];
                len_route = len_route + 2;
                for t = 1:numel(q)
                    sat_list_t = rank_satellites(vrp2e, q(t), s_off);
                    seq_r(len_route+1:len_route+2) = [sat_list_t(1), q(t)];
                    len_route = len_route + 2;
                end
                q = [];
            end
            continue;
        end

        % 采纳最佳候选
        seq_take = best_route_candidates{best_idx};
        seq_r(1:numel(seq_take)) = seq_take;
        len_route = numel(seq_take);
        no_progress_round = 0;
    end

    seq_r(len_route+1:end) = [];
end


% ========================================================================
% 后悔插入（Regret insertion，队列式，不丢客户；无法插入时轮转并防死循环）
% ========================================================================
function seq_r = greedyinr2(vrp2e, route, seq_c, s_off)
% Regret insertion (带时间窗+续航预检，并避免 seq_c / s_cus 长度不一致问题)
% - seq_c: 待插入的客户节点（全局编号 > num_sat）
% - route: 当前二级串（含卫星分隔符），形如 [s, c1, c2, s, c3, ...]
% - s_off: 禁用的卫星集合（可为空）
%
% 设计要点：
%   1) 每一轮都根据当前 seq_c 重新计算“开新回路的最佳卫星”，避免并行数组不同步
%   2) 若某个客户在所有位置都不可行，会被“轮转到队尾”尝试下一轮；
%      若连续多轮无进展，则强制把该客户以“新开回路”的方式插入，保证不丢客户、也不死循环
%
    fleet   = vrp2e.fleet;
    dis_sc  = vrp2e.dis_sc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    bigM = 1e8;

    % 统一为行向量
    route = route(:)'; 
    seq_c = seq_c(:)';

    len_route = numel(route);
    seq_r = zeros(1, 2*num_cus);
    if len_route > 0
        seq_r(1:len_route) = route;
    end

    % 允许的最大“无进展轮转次数”，超过则强制插入一个客户避免死循环
    max_stall = max(50, 3*numel(seq_c));
    stall_cnt = 0;

    while ~isempty(seq_c)

        % ---------- 重新计算：每个客户新开回路的最佳卫星 ----------
        num_c = numel(seq_c);
        s_cus = zeros(1, num_c);
        approx_cost = zeros(1, num_c);
        for k = 1:num_c
            c = seq_c(k);
            cost_newr = 2 * dis_sc(c, 1:num_sat);
            if ~isempty(s_off)
                cost_newr(s_off) = inf;
            end
            [v, sat] = min(cost_newr);
            s_cus(k) = sat;
            approx_cost(k) = v;
        end

        % ---------- 若当前还没有任何回路，先开一条 ----------
        if len_route == 0
            % 保持你原来的选择：approx_cost 最大者优先（通常是“更难插”的先处理）
            [~, idx_sel] = max(approx_cost);
            c_sel = seq_c(idx_sel);
            s_sel = s_cus(idx_sel);

            cand = [s_sel, c_sel];

            % 即便不满足 TW/续航，也强制生成一条回路，避免丢客户
            seq_r(1:2) = cand;
            len_route = 2;

            % 从池中移除
            seq_c(idx_sel) = [];
            stall_cnt = 0;
            continue;
        end

        % ---------- 常规 regret：计算每个客户在各位置的代价（只对可行位置计值） ----------
        route_now = seq_r(1:len_route);
        num_pos = len_route + 1;          % 插入位置个数（含“新开回路”作为最后一个位置）
        cost_c = inf(num_c, num_pos);

        for idx = 1:num_c
            c = seq_c(idx);
            s = s_cus(idx);

            % 1) 插入到现有串的不同位置（从 2 开始，避免插到最前面破坏“起始卫星”）
            for pos = 2:num_pos
                route_cand = [route_now(1:pos-1), c, route_now(pos:end)];
                if ~route_tw_feasible(vrp2e, route_cand)
                    continue;
                end
                [equip2, energy2, c_weight, c_energy] = msecond(vrp2e, route_cand);
                num_fleet_cand = sum(route_cand <= num_sat);
                c_fleet = max(num_fleet_cand - fleet(2,2), 0);
                cost_c(idx, pos-1) = equip2 + energy2 + bigM*(c_weight + c_energy + c_fleet);
            end

            % 2) 新开一条回路（在末尾追加 [s, c]）
            route_new = [route_now, s, c];
            if route_tw_feasible(vrp2e, route_new)
                [equip2, energy2, c_weight, c_energy] = msecond(vrp2e, route_new);
                num_fleet_cand = sum(route_new <= num_sat);
                c_fleet = max(num_fleet_cand - fleet(2,2), 0);
                cost_c(idx, num_pos) = equip2 + energy2 + bigM*(c_weight + c_energy + c_fleet);
            end
        end

        % ---------- regret 选择 ----------
        [val_cost, srt_pos] = sort(cost_c, 2, 'ascend');

        if num_pos == 1
            regret = zeros(num_c,1);
        elseif num_pos == 2
            regret = val_cost(:,2) - val_cost(:,1);
        else
            regret = sum(val_cost(:,2:min(3,num_pos)),2) - 2*val_cost(:,1);
        end

        [~, idx_sel] = max(regret);
        best_pos_index = srt_pos(idx_sel, 1);
        c_sel = seq_c(idx_sel);
        s_sel = s_cus(idx_sel);

        % ---------- 如果该客户所有位置都不可行：轮转或强制插入 ----------
        if isinf(val_cost(idx_sel,1))
            stall_cnt = stall_cnt + 1;

            if stall_cnt >= max_stall
                % 强制：在末尾新开一条回路（哪怕不可行），保证进度
                seq_r(len_route+1:len_route+2) = [s_sel, c_sel];
                len_route = len_route + 2;

                seq_c(idx_sel) = [];
                stall_cnt = 0;
            else
                % 轮转到队尾，下一轮再尝试
                seq_c = [seq_c(1:idx_sel-1), seq_c(idx_sel+1:end), c_sel];
            end
            continue;
        end

        % ---------- 执行插入（可行） ----------
        if best_pos_index <= len_route
            pos_ins = best_pos_index + 1;  % 注意 cost_c 的列 (pos-1)
            seq_r(pos_ins+1:len_route+1) = seq_r(pos_ins:len_route);
            seq_r(pos_ins) = c_sel;
            len_route = len_route + 1;
        else
            % 新开回路
            seq_r(len_route+1:len_route+2) = [s_sel, c_sel];
            len_route = len_route + 2;
        end

        % 从池中移除
        seq_c(idx_sel) = [];
        stall_cnt = 0;
    end

    seq_r(len_route+1:end) = [];
end


% ========================================================================
% 选卫星工具：按距离排序，过滤 s_off；并返回第一个使 [s,c] 可行的卫星
% ========================================================================
function sat_list = rank_satellites(vrp2e, c, s_off)
    num_sat = vrp2e.num_sat;
    dis_sc  = vrp2e.dis_sc;
    cost = 2 * dis_sc(c, 1:num_sat);
    if ~isempty(s_off)
        cost(s_off) = inf;
    end
    [~, sat_list] = sort(cost, 'ascend');
    sat_list = sat_list(isfinite(cost(sat_list)));
    if isempty(sat_list)
        sat_list = 1:num_sat; % 极端兜底：不应发生
    end
end

function [s_sel, ok] = first_feasible_sat(vrp2e, c, sat_list)
    ok = false;
    s_sel = sat_list(1);
    for t = 1:numel(sat_list)
        s = sat_list(t);
        if route_tw_feasible(vrp2e, [s, c])
            s_sel = s;
            ok = true;
            return;
        end
    end
end


% ========================================================================
% 以下 route_tw_feasible / energy_segment / init_load / update_load
% 你原来那套保持不变（直接沿用你文件后半部分即可）
% ========================================================================

% 时间窗可行性检查（首客户延后起飞、末尾返回最近卫星、速度以 vrp2e.V 计）
% 时间窗可行性检查（支持“多条回路串联”的 seq；每条回路首客户 wait=0；末尾返回最近卫星）

% ========================================================================
% 时间窗可行性预检（仅检查 TW，不在此处做能耗/载重约束；能耗由 msecond 统一判定）
% 规则：每条子回路的首客户允许“延后起飞”，因此首客户不需要等待（仅时间对齐，不计能耗）
% 注意：msecond 内部会按“最后客户返回最近卫星”闭合能耗；TW 这里无需显式加回程。
% ========================================================================
function feasible = route_tw_feasible(vrp2e, seq)
    num_sat = vrp2e.num_sat;
    feasible = true;
    if isempty(seq), return; end
    seq = seq(:)';

    % 提取时间窗（优先 Windows；否则 tw_start/tw_end）
    [tw_s, tw_e, hasTW] = get_tw(vrp2e);

    % service：若提供节点维度则用节点索引；若提供客户维度则用 (node-num_sat)
    service = [];
    if isfield(vrp2e,'service') && ~isempty(vrp2e.service)
        service = vrp2e.service(:);
    end

    loc = find(seq <= num_sat);
    if isempty(loc)
        % 没有卫星断点：当作单条回路不合法（你的编码里每段都必须以卫星开头）
        feasible = false;
        return;
    end

    for k = 1:numel(loc)
        st = loc(k);
        if k < numel(loc), ed = loc(k+1)-1; else, ed = numel(seq); end
        if ed <= st
            continue; % 空回路不判 infeasible，后续会被清理
        end
        sub = seq(st:ed);

        if ~route_tw_feasible_single(vrp2e, sub, tw_s, tw_e, hasTW, service)
            feasible = false;
            return;
        end
    end
end

function ok = route_tw_feasible_single(vrp2e, route, tw_s, tw_e, hasTW, service)
    % route: [sat, customer, customer, ...] 或末尾可能带卫星（若带卫星则忽略尾卫星）
    num_sat = vrp2e.num_sat;
    V       = vrp2e.V;
    dis_sc  = vrp2e.dis_sc;

    ok = true;
    if isempty(route) || numel(route) < 2
        return;
    end

    % 若末尾是卫星，忽略（open route）
    if route(end) <= num_sat
        route = route(1:end-1);
        if numel(route) < 2
            return;
        end
    end

    sat = route(1);
    seqCus = route(2:end);
    if any(seqCus <= num_sat)
        % 子回路内部不应出现卫星（除非作为断点），视为不可行
        ok = false; return;
    end

    % travel time minutes
    travel_min = @(i,j) dis_sc(i,j) / max(1e-12, V) * 60;

    % service time minutes (node-index aware)
    serv = @(node) get_service_minutes(node, num_sat, service);

    % -------- 首客户：允许延后起飞 => 抵达时间对齐到 max(arrival, tw_start) --------
    t = 0; % 卫星起飞基准
    prev = sat;
    z0 = seqCus(1);
    t_arr = t + serv(prev) + travel_min(prev, z0);

    if hasTW
        idx0 = z0 - num_sat;
        if idx0 < 1 || idx0 > numel(tw_s)
            ok = false; return;
        end
        if t_arr > tw_e(idx0) + 1e-9
            ok = false; return;
        end
        % 延后起飞使得首客户无等待：把时刻对齐到 tw_start
        t = max(t_arr, tw_s(idx0));
    else
        t = t_arr;
    end
    prev = z0;

    % -------- 后续客户：正常推进，早到则等待到 tw_start（时间对齐） --------
    for kk = 2:numel(seqCus)
        z = seqCus(kk);
        t_arr = t + serv(prev) + travel_min(prev, z);

        if hasTW
            idx = z - num_sat;
            if t_arr > tw_e(idx) + 1e-9
                ok = false; return;
            end
            t = max(t_arr, tw_s(idx));
        else
            t = t_arr;
        end
        prev = z;
    end
end

function [tw_s, tw_e, hasTW] = get_tw(vrp2e)
    hasTW = false;
    tw_s = [];
    tw_e = [];
    if isfield(vrp2e,'Windows') && ~isempty(vrp2e.Windows)
        W = vrp2e.Windows;
        if size(W,2) >= 2
            tw_s = W(:,1);
            tw_e = W(:,2);
            hasTW = true;
            return;
        end
    end
    if isfield(vrp2e,'tw_start') && isfield(vrp2e,'tw_end') && ~isempty(vrp2e.tw_start)
        tw_s = vrp2e.tw_start(:);
        tw_e = vrp2e.tw_end(:);
        hasTW = true;
    end
end

function smin = get_service_minutes(node, num_sat, service)
    % service 可能是：
    % 1) 节点维度 (sat+cus)：index = node
    % 2) 客户维度 (cus)：index = node-num_sat
    % 3) 缺失：0
    if isempty(service)
        smin = 0;
        return;
    end
    if numel(service) >= node
        % 节点维度
        smin = service(node);
    else
        idx = node - num_sat;
        if idx >= 1 && idx <= numel(service)
            smin = service(idx);
        else
            smin = 0;
        end
    end
end
