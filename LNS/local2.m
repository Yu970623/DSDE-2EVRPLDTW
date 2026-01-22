%% 第二层路径的局部搜索----
function seq_c = local2(vrp2e,seq_c)
    %----5种局部搜索----
%     seq_c = split2(vrp2e,seq_c);
    seq_c = move2(vrp2e,seq_c); %随机删除所有客户并重新插入完整路段对比结果
    seq_c = swap2(vrp2e,seq_c); %随机将所有客户与其单一+连续邻居交换后对比结果
    seq_c = opt2(vrp2e,seq_c);  %每个子路段内，遍历逆转部分子路段
    seq_c = opt2x(vrp2e,seq_c); %拼接同一卫星下不同子路段内的子路段
    %----end----
end

%% --------------split2策略:第二层路径优化--------------------
function seq_cs = split2(vrp2e,route)
    fleet = vrp2e.fleet;
    demand = vrp2e.demand;  
    dis_sc = vrp2e.dis_sc;  
    num_sat = vrp2e.num_sat;  

    len_route = numel(route);
    seq_cs    = zeros(1,len_route+fleet(2,2));
    rg        = 0;
    %----卫星s的所有路径----
    sat_fleet     = route(route<=num_sat);                      %车辆对应的卫星
    sign_fleet    = cumsum(route<=num_sat,2);                    %卫星和客户对应的车辆
    for s = 1:num_sat
        sign_fleet_s        = find(sat_fleet==s);                 %卫星s对应的车辆号码
        [logic_loc,~]       = ismember(sign_fleet,sign_fleet_s);  %卫星s及对应客户参数为1
        cus_sat             = route(logic_loc);                   %卫星s的所有路径（含s）
        route_s             = cus_sat(cus_sat>num_sat);                %提取路径中的客户点（不含s）
        len_s               = numel(sign_fleet_s);
        if len_s >1
            seq_c           = splitopt(dis_sc(s,route_s),dis_sc(route_s,route_s),demand(route_s),fleet(2,1));
            loc_c           = find(seq_c>0);
            seq_c(loc_c)    = route_s(seq_c(loc_c));
            seq_c(seq_c==0) = s;
            len_r           = numel(seq_c);
            seq_cs(rg+1:rg+len_r) = seq_c;
            rg              = rg+len_r;
        else
            len_r           = numel(route_s)+1;
            seq_cs(rg+1:rg+len_r) = [s,route_s];
            rg              = rg+len_r;
        end
    end
    seq_cs(rg+1:end) = [];
end
% ----按照split算法对路径进行重新分割----
 function seq_c = splitopt(dis_sc,dis_cc,require,capcity)
    nc        = length(dis_sc);                  %被选取的卫星数目
    point     = zeros(1,nc);                    %每个客户的前向指针
    cost_c    = inf*ones(1,nc);                 %到当前客户需要的路径长度
    for i = 1:nc
        load  = 0;                              %当前的车载容量
        cost  = 2*dis_sc(i);                    %当前的路径长度
        j     = i;
        while j<=nc && load<=capcity
            load  = load+require(j);
            if j>i
                cost = cost-dis_sc(j-1)+dis_cc(j-1,j)+dis_sc(j); %依次插入卫星代价
            end
            if load<=capcity  %容量是否满足判断
                if i-1 ==0
                    cost_c(j) = cost;
                    point(j)  = i-1;
                elseif cost_c(i-1)+cost<cost_c(j)  %车辆长度代价比较
                    cost_c(j) = cost_c(i-1)+cost;
                    point(j)  = i-1;
                end
                j = j+1;
            end
        end
    end
    %----根据指针在路径中插入卫星点----
    seq_c = [1:nc,zeros(1,nc)];
    r     = 1;                        %统计路径的数目
    i     = point(nc);                %插入卫星点的位置
    seq_c(i+2:end) = seq_c(i+1:end-1);
    seq_c(i+1) = 0;
    while i>0
        i = point(i);
        r = r+1;
        seq_c(i+2:end) = seq_c(i+1:end-1);
        seq_c(i+1) = 0;
    end
    seq_c(nc+r+1:end) = [];
 end
 
%% --------------------------2-opt策略:第二层路径优化(同一路径中的卫星优化)-------------------------
function seq_cs = opt2(vrp2e,seq_cs)
    num_sat = vrp2e.num_sat;
    loc_s = [find(seq_cs<=num_sat),numel(seq_cs)+1];         %卫星对应的序号
    nfs   = length(loc_s)-1;                            %车辆总数 
    for i = 1:nfs
        if loc_s(i+1)-loc_s(i)>3                        %需要优化的条件：客户数目>2
           seq_cs(loc_s(i):loc_s(i+1)-1) = opttwo(vrp2e,[seq_cs(loc_s(i):loc_s(i+1)-1),seq_cs(loc_s(i))]);  %逐一对每趟旅程中的客户进行交换
        end
    end
end
%------------采用2-opt策略将同一条路径重组------------------
function route = opttwo(vrp2e,route)  %逆转部分子路段
    Energy_distribution = funcopt2(vrp2e,route);
    len_route = numel(route);   %客户数目
    i         = 2;
    while i <= len_route-2
        j = i+1;
        while j <= len_route-1
            new_route = [route(1:i-1), route(j:-1:i), route(j+1:end)];
            newEnergy_distribution = funcopt2(vrp2e,new_route);
            diff_dis = sum(Energy_distribution)-sum(newEnergy_distribution);  %计算新路径的总能耗
            if diff_dis > 0
                Energy_distribution = newEnergy_distribution;
                route(i:j) = route(j:-1:i); 
                i = 2;
                j = i+1;
            else
                j = j+1;
            end
        end
        i = i+1;
    end
    route = route(1:end-1);              %生成新的路径
end
%% --------------------------2-opt*策略:第二层路径优化-------------------------
function seq_cs = opt2x(vrp2e,seq_cs)
    num_sat = vrp2e.num_sat;
    
    len_route   = numel(seq_cs);
    loc_s = [find(seq_cs<=num_sat),len_route+1];              %卫星对应的序号
    nfs     = length(loc_s)-1;                   %车辆总数
    route   = cell(nfs,1);                       %将每条路径独立的存储
    for i = 1:nfs
        route{i} = [seq_cs(loc_s(i):loc_s(i+1)-1),seq_cs(loc_s(i))];
    end
    %----2-opt*----
    for i = 1:nfs-1
        for j = i+1:nfs
            if route{i}(1)==route{j}(1) && numel(route{i})>2 && numel(route{j})>2
               [route{i},route{j}] = opttwox(vrp2e,route{i},route{j});  %注意交换同卫星下出发的路段中的客户
            end
        end
    end
    %----路径重组----
    r = 0;
    for i = 1:nfs
        len = numel(route{i})-1;
        if len>1
            seq_cs(r+1:r+len) = route{i}(1:len);  %更新客户与卫星序列
            r = r+len;
        end
    end
    seq_cs(r+1:end) = [];
end
%------------采用2-opt*策略将同一卫星的两条路径重组------------------
function [seq_c1,seq_c2] = opttwox(vrp2e,seq_c1,seq_c2)
    nc1 = numel(seq_c1);
    nc2 = numel(seq_c2);
    fr  = funcopt2(vrp2e,seq_c1)+funcopt2(vrp2e,seq_c2);  %计算原始两条路径的总距离
    i   = 1;
    while i <= nc1-1  %使用变量i遍历第一条路径seq_c1中的所有客户，但不包括最后一个客户
        j = 1;
        while j <= nc2-1    %使用变量j遍历第二条路径seq_c2中的所有客户，同样不包括最后一个客户
            %----第一种组合方式----
            ra1 = [seq_c1(1:i),seq_c2(j+1:end)];  %将seq_c1的前i个客户与seq_c2的从j+1到末尾的客户组合成新的路径ra1
            ra2 = [seq_c2(1:j),seq_c1(i+1:end)];  %将seq_c2的前j个客户与seq_c1的从i+1到末尾的客户组合成新的路径ra2
            fra    = funcopt2(vrp2e,ra1)+funcopt2(vrp2e,ra2);  %成本计算方法，包括约束违反情况
            %----第二种组合方式----
            rb1 = [seq_c1(1:i),seq_c2(j:-1:1)];  %将seq_c1的前i个客户与seq_c2的从j到第一个客户逆序组合成新的路径rb1
            rb2 = [seq_c1(end:-1:i+1),seq_c2(j+1:end)];  %将seq_c1的从最后一个客户逆序到i+1的客户与seq_c2的从j+1到末尾的客户组合成新的路径rb2
            frb    = funcopt2(vrp2e,rb1)+funcopt2(vrp2e,rb2);  %成本计算方法，包括约束违反情况
            %----判断优劣---- 每种组合方式计算新的总距离（fra和frb）
            if fra <= frb
                if fra < fr
                    seq_c1 = ra1;
                    seq_c2 = ra2;
                    fr     = fra;
                    i      = 1;
                    j      = 0;
                end
            else
                if frb < fr
                    seq_c1 = rb1;
                    seq_c2 = rb2;
                    fr     = frb;
                    i      = 1;
                    j      = 0;
                end
            end
            nc1    = numel(seq_c1);
            nc2    = numel(seq_c2);
            j = j+1;    
       end
       i = i+1;    
    end
end
 %% --------------------------move2策略:第二层路径优化-------------------------
function route = move2(vrp2e,route)
    neib_cc = vrp2e.neib_cc;
    num_sat = vrp2e.num_sat;  
    num_cus = vrp2e.num_cus;
    tao     = min(25,num_cus-1);
    %----待插入的客户随机排序----
    seq_c   = randperm(num_cus)+num_sat;   %将客户从一个位置移动到另一个位置来改善路径

    for c = seq_c
        loc_c      = find(route==c);                            %删除客户所在解中的位置
        %----计算原客户的代价----
        cost_init = funcopt4(vrp2e,route);
        seq_r         = [route(1:loc_c-1),route(loc_c+1:end)];
        neib_c        = neib_cc(c-num_sat,2:tao+1);               %邻居客户
        [~,pos_in]    = ismember(neib_c,seq_r);                   %邻居客户在路径中的位置（含自身）
% ---- 关键修复：过滤掉不在 seq_r 里的邻居（ismember 返回 0）----
        pos_in = pos_in(pos_in > 0);
        pos_in = unique(pos_in,'stable');
        if isempty(pos_in)
            continue;   % 没有可参考的邻居位置，跳过这个客户 c
        end
        % 计算插入到 rfront 后的新路径
        c_front = arrayfun(@(k) [seq_r(1:k-1), c, seq_r(k:end)], pos_in, 'UniformOutput', false);
        cost_front = arrayfun(@(k) [funcopt4(vrp2e,c_front{k})], 1:length(c_front), 'UniformOutput', false);
        % 计算插入到 rback 后的新路径
        c_back = arrayfun(@(k) [seq_r(1:k), c, seq_r(k+1:end)], pos_in, 'UniformOutput', false);
        cost_back = arrayfun(@(k) [funcopt4(vrp2e,c_back{k})], 1:length(c_back), 'UniformOutput', false);
        [cost_c,loc]  = min([cell2mat(cost_front);cell2mat(cost_back)],[],1); %cost_c存储插入邻居X前或后最小成本，loc存储插入邻居X前还是后
        [cost_new,loc_in] = min(cost_c);  %最小邻居的成本和该邻居序号
        if cost_new < cost_init
            loc_in       = pos_in(loc_in)+loc(loc_in)-2;
            route        = [seq_r(1:loc_in),c,seq_r(loc_in+1:end)];
        end
    end
    %----消除空路径----
    len_route   = numel(route);                                 %已有的路径长度
    loc_sat     = [find(route<=num_sat),len_route+1];           %卫星在路径中的位置
    route(loc_sat(loc_sat(2:end)-loc_sat(1:end-1)==1)) = [];    %删除空车从所在路径    
end

%% --------------------------swap2策略:第二层路径优化-------------------------
function route = swap2(vrp2e,route)
    % 说明：原始 swap2 里用 find(route==c) 可能返回多个位置/空，
    % 直接做向量赋值会触发“左右元素数目不同”的报错。
    % 这里改成：只在“同一条子回路(同一卫星段)”内部交换两个客户的单一位置，
    % 并且先做时间窗+续航可行性预检（含悬停等待能耗）。

    neib_cc = vrp2e.neib_cc;
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    tao = min(25, num_cus-1);

    % 随机顺序遍历客户（客户节点编号：num_sat+1 ... num_sat+num_cus）
    seq_c = randperm(num_cus) + num_sat;

    for c1 = seq_c
        loc1 = find(route==c1, 1, 'first');
        if isempty(loc1)
            continue;
        end

        % 确定 c1 所属的子回路（以卫星为分段符号）
        sign_fleet = cumsum(route<=num_sat,2);
        rid = sign_fleet(loc1);
        idx_route = find(sign_fleet==rid);           % 该子回路在全串中的索引
        sub = route(idx_route);                      % 子回路串（形如 [s, c, c, ...]）

        % c1 在子回路中的位置
        pos1 = find(sub==c1, 1, 'first');
        if isempty(pos1) || pos1==1
            continue; % pos1==1 不应发生（1 是卫星）
        end

        base_cost = funcopt1(vrp2e, sub);

        % 仅尝试在同一子回路里，与“邻居列表中也在该子回路内”的客户交换
        improved = false;
        for i = 2:(tao+1)
            c2 = neib_cc(c1-num_sat, i);
            pos2 = find(sub==c2, 1, 'first');
            if isempty(pos2) || pos2==1 || pos2==pos1
                continue;
            end

            sub2 = sub;
            sub2([pos1,pos2]) = sub2([pos2,pos1]);

            % 可行性预检（支持单条回路）
            if ~route_tw_feasible(vrp2e, sub2)
                continue;
            end

            new_cost = funcopt1(vrp2e, sub2);
            if new_cost < base_cost
                route(idx_route) = sub2;  % 接受
                sub      = sub2;
                base_cost = new_cost;
                pos1     = pos2;          % c1 位置更新
                improved = true;
                break;                    % 先接受一次改进，跳出
            end
        end

        if improved
            % 接受后继续下一个客户（也可以多次接受，但这里保守一点）
            continue;
        end
    end
end


function fvrp = funcopt1(vrp2e,route)
    if ~route_tw_feasible(vrp2e, route)
penalty = 1e8;
        % 若不满足时间窗，直接返回巨大惩罚（等价于不可行）
        fvrp = penalty;
        return;
    end
    [equip_cost2,energy_cost2,c_weight,c_energy] = msecond(vrp2e,route);
    penalty = 1e8;
    fvrp = equip_cost2 + energy_cost2 + penalty*(c_weight + c_energy);
end
function fvrp = funcopt2(vrp2e,route)
    % 评价函数：用于 local2 中选择更优插入/交换
    % 只依赖二级自身信息（不依赖一级到达时刻），因此调用 msecond 的 2 参数版本（近似但一致）
    if isempty(route)
        fvrp = 0; 
        return;
    end
    if ~route_tw_feasible(vrp2e, route)
        fvrp = 1e8;
        return;
    end

    % route 可能以 sat 闭合：若末尾是 sat，则传入 open route 以避免多算一段
    num_sat = vrp2e.num_sat;
    if route(1) <= num_sat && route(end) <= num_sat
        route_open = route(1:end-1);
    else
        route_open = route;
    end

    [equip_cost2,energy_cost2,c_weight,c_energy] = msecond(vrp2e,route_open);
    penalty = 1e8;
    fvrp = equip_cost2 + energy_cost2 + penalty*(c_weight + c_energy);
end
function fvrp = funcopt4(vrp2e,route)
    % 评价函数：route 可能未显式闭合；这里直接交给 msecond（其内部按“最后客户返回最近卫星”闭合）
    if isempty(route)
        fvrp = 0;
        return;
    end
    % 若 route 不以卫星开头，则保守判不可行
    if route(1) > vrp2e.num_sat
        fvrp = 1e8;
        return;
    end

    if ~route_tw_feasible(vrp2e, route)
        fvrp = 1e8;
        return;
    end

    % 若末尾是 sat 闭合则去掉最后 sat，否则原样
    num_sat = vrp2e.num_sat;
    if route(end) <= num_sat
        route_open = route(1:end-1);
    else
        route_open = route;
    end

    [equip_cost2,energy_cost2,c_weight,c_energy] = msecond(vrp2e,route_open);
    penalty = 1e8;
    fvrp = equip_cost2 + energy_cost2 + penalty*(c_weight + c_energy);
end
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
