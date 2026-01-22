function pulish = time_violate(seq_sat, route, data, vrp2e)
% time_violate (ALNS)
% 计算两级配送时序下的时间窗违反惩罚（第二层“可等待”，不计能耗，仅计迟到惩罚）
%
% 关键时序假设（与 msecond/insert_cus 对齐）：
%   1) 无人机可起飞时刻 >= 车辆抵达卫星并完成卸货服务的时刻；
%   2) 每条无人机子路段的首客户允许“延后起飞”以对齐其最早时间窗，因此首客户不产生悬停等待；
%      但若车辆到达卫星过晚，则首客户将发生“迟到”，并计入惩罚；
%   3) 后续客户若早到则等待至其最早时间窗（仍可行），若晚到则计入惩罚。
%
% 输入：
%   seq_sat : 一级路径（仓库/站点序列，卫星以 num_dep+sat 编码）
%   route   : 二级路径（卫星(1..num_sat) + 客户(num_sat+1..num_sat+num_cus) 串接）
%   data    : 需包含 T1,T2,Services,Windows,CW
%   vrp2e   : 需包含 num_sat,num_cus,rad_ds,V (速度同 data.T2 的单位保持一致)
%
% 输出：
%   pulish  : 时间窗违反的加权惩罚（>=0）

    % -------- 0. 读取与健壮性 --------
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    if isempty(route) || isempty(seq_sat)
        pulish = 0;
        return;
    end

    T1      = data.T1;        %#ok<NASGU> % 这里仍按原逻辑计算 time_first（若你不需要一级时序，可替换）
    T2      = data.T2;        % (num_sat+num_cus) x num_cus  或同等索引方式
    service = data.Services(:);
    windows = data.Windows;
    CW      = data.CW;

    num_dep = size(vrp2e.rad_ds, 1);

    % -------- 1. 计算一级到达时刻 time_first（每个 seq_sat 位置的到达时刻）--------
    % time_first(i) = 抵达 seq_sat(i) 的时刻（分钟）
    time_first = zeros(1, length(seq_sat));

    loc_dep = find(seq_sat <= num_dep);
    if isempty(loc_dep)
        % 若没有显式仓库节点，默认从序列第1个节点开始
        loc_dep = 1;
    end

    for ii = 1:length(loc_dep)
        start_idx = loc_dep(ii);
        if ii < length(loc_dep)
            end_idx = loc_dep(ii+1) - 1;
        else
            end_idx = length(seq_sat);
        end

        for j = start_idx:end_idx
            if j == start_idx
                time_first(j) = 0;
            elseif j == start_idx + 1
                prev_node    = seq_sat(j-1);
                current_node = seq_sat(j);
                time_first(j) = time_first(j-1) + data.T1(prev_node, current_node - num_dep);
            else
                prev_node    = seq_sat(j-1);      % 这里 prev_node 一定是卫星（num_dep+sat）
                current_node = seq_sat(j);
                sat_prev = prev_node - num_dep;
                sat_prev = max(1, sat_prev);      % 防御
                time_first(j) = time_first(j-1) + service(sat_prev) + data.T1(prev_node, current_node - num_dep);
            end
        end
    end

    % -------- 1.1 预计算每个卫星的“可起飞时刻” sat_ready(s) --------
    % sat_ready(s) = min_{访问该卫星的位置 p} (time_first(p) + service(s))
    sat_ready = inf(1, num_sat);

    pos_sat = find(seq_sat > num_dep);
    if ~isempty(pos_sat)
        sat_ids = seq_sat(pos_sat) - num_dep;          % 1..num_sat (理想情况)
        ok = (sat_ids >= 1) & (sat_ids <= num_sat);
        sat_ids = sat_ids(ok);
        pos_sat = pos_sat(ok);

        if ~isempty(sat_ids)
            ready_vals = time_first(pos_sat) + service(sat_ids)';
            % accumarray + min
            tmp = accumarray(sat_ids(:), ready_vals(:), [num_sat, 1], @min, inf);
            sat_ready = tmp(:)';
        end
    end

    if any(isinf(sat_ready(route(route <= num_sat))))
        % 二级路径使用了一级未服务到的卫星，直接给一个较大的违反值，避免 NaN/Inf 传播
        pulish = 1e6;
        return;
    end

    % -------- 2. 计算二级路径访问时刻 time_second（与 route 同长）--------
    time_second = zeros(1, length(route));

    loc_sat = find(route <= num_sat);    % 二级路径里每条子路段的起始卫星位置
    if isempty(loc_sat)
        pulish = 1e6;
        return;
    end

    for r = 1:length(loc_sat)
        start_idx = loc_sat(r);
        if r < length(loc_sat)
            end_idx = loc_sat(r+1) - 1;
        else
            end_idx = length(route);
        end

        s = route(start_idx);  % 起始卫星 id
        t = sat_ready(s);      % 卫星可起飞时刻（分钟）
        time_second(start_idx) = t;

        % 子路段客户序列
        for j = start_idx+1:end_idx
            prev_node = route(j-1);
            cur_node  = route(j);

            if cur_node <= num_sat
                % 正常情况下子路段内部不应出现卫星；若出现则直接继承
                time_second(j) = time_second(j-1);
                continue;
            end

            cus = cur_node - num_sat; % 1..num_cus
            e_to = windows(cus, 1);
            l_to = windows(cus, 2);

            if prev_node <= num_sat
                % 从卫星出发：不再重复加 service(卫星)，因为 sat_ready 已包含
                t_arr = time_second(j-1) + T2(prev_node, cus);
            else
                % 客户->客户：加上前一客户服务时间
                t_arr = time_second(j-1) + service(prev_node) + T2(prev_node, cus);
            end

            % 早到则等待（不记惩罚），晚到会在第 3 部分记惩罚
            if t_arr < e_to
                time_second(j) = e_to;
            else
                time_second(j) = t_arr;
            end

            % 若已经明显超过 latest，可提前结束这一子路段的传播（但仍要统计惩罚）
            if time_second(j) > l_to + 1e6
                % 防止异常数值爆炸
                time_second(j) = l_to + 1e6;
            end
        end
    end

    % -------- 3. 时间窗违反惩罚（只计迟到，早到已等待）--------
    late = zeros(num_cus, 1);
    for i = 1:length(route)
        node = route(i);
        if node > num_sat
            cus = node - num_sat;
            l_to = windows(cus, 2);
            if time_second(i) > l_to
                late(cus) = max(late(cus), time_second(i) - l_to);
            end
        end
    end

    % 按原逻辑加权（若 CW 是 1x2 或 num_cus x 2 的形式，也做兼容）
    if isvector(CW)
        if numel(CW) == 1
            pulish = CW(1) * sum(late);
        else
            % 取第二列权重（迟到）
            pulish = CW(end) * sum(late);
        end
    else
        % CW(i,2) 对应迟到
        if size(CW,1) == num_cus && size(CW,2) >= 2
            pulish = sum(late .* CW(:,2));
        else
            pulish = sum(late);
        end
    end
end
