%% --------------------------初始化第二层（FAST 版，适用于大规模实例）--------------------------
% 目标：显著加速（优先速度，牺牲部分初始质量）
% 核心策略：
% 1) 先把客户分配给最近卫星（或按距离轮盘也可，这里用最近卫星更稳定）
% 2) 每个卫星内部按 tw_start（最早开始时间）升序排序
% 3) 逐个追加客户，若追加后子回路不满足【时间窗 + 续航(含等待悬停能耗, 首客户 wait=0)】则开新子回路
% 4) 输出 seq_cus 采用“开路表示”： [sat, c1, c2, ..., sat, c3, ...]（不强制闭合）
%
% 说明：本函数不依赖 msecond，也不做 regret / 全位置插入，因此 100 客户时不会“卡死”。

function [seq_cus,seq_sat,cap_sat] = initial(vrp2e)

    % ---------- 基本读取 ----------
    fleet   = vrp2e.fleet;
    demand  = vrp2e.demand;
    dis_ds  = vrp2e.dis_ds;
    dis_sc  = vrp2e.dis_sc;
    num_dep = length(vrp2e.rad_ds(:,1));
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    % ---------- 时间窗/服务（若缺失则退化为仅做续航检查） ----------
    hasTW = isfield(vrp2e,'tw_start') && isfield(vrp2e,'tw_end');
    if hasTW
        tw_start = vrp2e.tw_start(:);
        tw_end   = vrp2e.tw_end(:);
    else
        tw_start = [];
        tw_end   = [];
    end
    if isfield(vrp2e,'service') && ~isempty(vrp2e.service)
        service = vrp2e.service(:);
    else
        service = [];
    end

    % ---------- Step1: 客户分配给卫星（最近卫星） ----------
    sat_cus = zeros(1,num_cus);
    for c = 1:num_cus
        node = c + num_sat;
        [~, sat_cus(c)] = min(dis_sc(1:num_sat, node));
    end

    % ---------- Step2: 每个卫星内部构造若干无人机子回路 ----------
    seq_cus = [];
    cap_s   = zeros(1,num_sat);

    for s = 1:num_sat
        cus_nodes = find(sat_cus==s) + num_sat;   % 全局节点编号
        if isempty(cus_nodes)
            continue;
        end
        cap_s(s) = sum(demand(cus_nodes));

        % 排序规则：优先 tw_start，其次卫星距离
        if hasTW
            idx = cus_nodes - num_sat;
            [~,ord] = sortrows([tw_start(idx), dis_sc(s, cus_nodes)'], [1,2]);
        else
            [~,ord] = sort(dis_sc(s, cus_nodes), 'ascend');
        end
        cus_nodes = cus_nodes(ord);

        % 贪婪追加：若不可行则新开子回路
        cur = s;    % 当前子回路（开路表示，起点卫星）
        for k = 1:numel(cus_nodes)
            cnode = cus_nodes(k);
            if numel(cur)==1
                cand = [cur, cnode];
                cur  = cand;
            else
                cand = [cur, cnode];
                if subroute_feasible_fast(vrp2e, cand, tw_start, tw_end, service)
                    cur = cand;
                else
                    % 结束当前子回路，写入 seq_cus；然后为该客户开启新子回路
                    seq_cus = [seq_cus, cur]; %#ok<AGROW>
                    cur = [s, cnode];
                end
            end
        end
        % 写入最后一个子回路
        if numel(cur) > 1
            seq_cus = [seq_cus, cur]; %#ok<AGROW>
        end
    end

    % 若由于极端情况导致 seq_cus 为空，则保底：每个客户独立成路
    if isempty(seq_cus)
        for c = 1:num_cus
            node = c + num_sat;
            [~, s] = min(dis_sc(1:num_sat, node));
            seq_cus = [seq_cus, s, node]; %#ok<AGROW>
        end
    end

    % ---------- Step3: 第一层（仓库->卫星）构造（保持原 CW 合并逻辑） ----------
    seq_sat = zeros(1,2*fleet(1,2)+2*num_sat);
    cap_sat = zeros(1,2*fleet(1,2)+2*num_sat);

    % 仍然采用“卫星分配到仓库”+ algcw1（规模小，不是瓶颈）
    dep_sat = zeros(1,num_sat);
    for s = 1:num_sat
        dep_sat(s) = roulette(1./dis_ds(1:num_dep, s+num_dep), 1);
    end

    nrs = 0;
    for s = 1:num_sat
        while cap_s(s) >= fleet(1,1)
            nrs = nrs + 2;
            seq_sat(nrs) = s;
            cap_sat(nrs) = fleet(1,1);
            cap_s(s) = cap_s(s) - fleet(1,1);
        end
    end

    for d = 1:num_dep
        dep_s = find(dep_sat==d) + num_dep;
        route = algcw1(dis_ds(d,dep_s), dis_ds(dep_s,dep_s), cap_s, fleet(1,1));
        loc = find(route>0);
        route(loc) = dep_s(route(loc));
        cap_r = route;
        cap_r(loc) = cap_s(route(loc)-num_dep);
        route(route==0) = d;
        nr = numel(route);
        seq_sat(nrs+1:nrs+nr) = route;
        cap_sat(nrs+1:nrs+nr) = cap_r;
        nrs = nrs + nr;
    end
    seq_sat(nrs+1:end) = [];
    cap_sat(nrs+1:end) = [];
end


%% --------------------------子回路可行性（FAST）--------------------------
% route_open: [s, z1, z2, ...]（不含闭合卫星）
% 规则：
% - “首客户 wait=0”：无人机起飞时间从首客户最早时刻反推（但不得早于卫星就绪 sat_ready=0）
% - 后续客户若早到则等待，等待计入该段能耗（与 msecond/insert_cus 的对齐逻辑）
% - 回程：最后客户返回最近卫星（无等待）
function ok = subroute_feasible_fast(vrp2e, route_open, tw_start, tw_end, service)
    ok = true;
    num_sat = vrp2e.num_sat;
    V       = vrp2e.V;
    dis     = vrp2e.dis_sc;
    Emax    = vrp2e.E;

    if numel(route_open) < 2 || route_open(1) > num_sat
        ok = false; return;
    end

    seqCus = route_open(2:end);
    if any(seqCus <= num_sat)
        % 若 route_open 中混入卫星（多段），这里只检查“单段”
        ok = false; return;
    end

    % 初始载荷（delivery 装货，pickup 到点后取货）
    load = init_load_fast(vrp2e, seqCus, num_sat);

    C = (9.81^3) / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

    % ---------- 首客户：按最早时刻反推起飞（wait=0） ----------
    s  = route_open(1);
    z1 = seqCus(1);
    T01 = dis(s, z1) / max(1e-9, V) * 60; % min

    sat_ready = 0; % 初始化阶段不考虑一级到货，后续由 time_violate/主评价去约束

    if ~isempty(tw_start)
        idx1 = z1 - num_sat;
        e1 = tw_start(idx1); l1 = tw_end(idx1);
        if sat_ready + T01 > l1 + 1e-9
            ok = false; return;
        end
        depart = max(sat_ready, e1 - T01);    % 反推起飞时间
        t_arr1 = depart + T01;                % 实际到达（>=e1 or >e1 if sat_ready 限制）
        if t_arr1 > l1 + 1e-9
            ok = false; return;
        end
        t = t_arr1;
        if idx1 <= numel(service), t = t + service(idx1); end
    else
        depart = sat_ready;
        t = depart + T01;
    end

    % wait=0 => 段能耗只含飞行
    P = (5 + load)^(3/2) * C;
    E = P * (T01/60);

    % 更新载荷
    load = update_load_fast(vrp2e, load, z1, num_sat);

    % ---------- 后续客户 ----------
    prev = z1;
    for k = 2:numel(seqCus)
        z = seqCus(k);
        T = dis(prev, z) / max(1e-9, V) * 60;
        t_arr = t + T;
        wait = 0;
        if ~isempty(tw_start)
            idx = z - num_sat;
            e = tw_start(idx); l = tw_end(idx);
            if t_arr > l + 1e-9
                ok = false; return;
            end
            if t_arr < e
                wait = e - t_arr;
                t = e;
            else
                t = t_arr;
            end
            if idx <= numel(service), t = t + service(idx); end
        else
            t = t_arr;
        end

        P = (5 + load)^(3/2) * C;
        E = E + P * ((T + wait)/60);

        if E > Emax + 1e-9
            ok = false; return;
        end

        load = update_load_fast(vrp2e, load, z, num_sat);
        prev = z;
    end

    % ---------- 回到最近卫星（无等待） ----------
    d_back = min(dis(prev, 1:num_sat));
    T_back = d_back / max(1e-9, V) * 60;
    P = (5 + load)^(3/2) * C;
    E = E + P * (T_back/60);

    ok = (E <= Emax + 1e-9);
end

function load0 = init_load_fast(vrp2e, seqCus, num_sat)
    dem = vrp2e.demand(:);
    if ~isfield(vrp2e,'type') || isempty(vrp2e.type)
        load0 = sum(dem(seqCus));
        return;
    end
    is_pick = false(size(seqCus));
    for ii = 1:numel(seqCus)
        node = seqCus(ii);
        try
            tp = vrp2e.type{node-num_sat};
        catch
            tp = vrp2e.type(node-num_sat);
        end
        is_pick(ii) = strcmpi(tp,'pickup');
    end
    load0 = sum(dem(seqCus(~is_pick)));
end

function load = update_load_fast(vrp2e, load, node, num_sat)
    dem = vrp2e.demand(node);
    if isfield(vrp2e,'type') && ~isempty(vrp2e.type) && node > num_sat
        try
            tp = vrp2e.type{node-num_sat};
        catch
            tp = vrp2e.type(node-num_sat);
        end
        if strcmpi(tp,'pickup')
            load = load + dem;
        else
            load = load - dem;
        end
    else
        load = load - dem;
    end
end



% Clarke-Wright for first echelon (same logic as old code)
function rount = algcw1(dis_dtos,dis_stos,demand_sat,capacity)
    nc         = length(dis_dtos);     %客户总数(这里对应卫星数)
    link       = zeros(2,nc);
    nlink      = zeros(1,nc);
    seq_veh    = 1:nc;
    cw_cc      = reshape(triu(bsxfun(@plus,dis_dtos,dis_dtos')-dis_stos+0.01,1),1,[]);
    [~,srt_cp] = sort(cw_cc,'descend');
    npair      = nc*(nc-1)/2;
    srt_cp     = srt_cp(1:npair);
    cr         = mod(srt_cp-1,nc)+1;
    cl         = ceil(srt_cp/nc);
    for i = 1:npair
        nv1 = seq_veh(cr(i));
        nv2 = seq_veh(cl(i));
        if nlink(cr(i))<2 && nlink(cl(i))<2 && nv1~= nv2 && demand_sat(nv1)+demand_sat(nv2) <= capacity 
            demand_sat(nv1) = demand_sat(nv1)+demand_sat(nv2);
            demand_sat(nv2) = 0;
            seq_veh(seq_veh==nv2) = nv1;
            nlink(cr(i)) = nlink(cr(i))+1;
            nlink(cl(i)) = nlink(cl(i))+1;
            if link(1,cr(i))== 0
                link(1,cr(i)) = cl(i);
            else
                link(2,cr(i)) = cl(i);
            end
            if link(1,cl(i))== 0
                link(1,cl(i)) = cr(i);
            else
                link(2,cl(i)) = cr(i);
            end
        end  
    end
    rount   = zeros(1,2*nc+1);
    cus_isolate = find(nlink==0);
    count = 2*numel(cus_isolate);
    if count>0
        rount(2:2:count) = cus_isolate;
    end
    cus_start = find(nlink==1);
    for nv = cus_start
        if sum(rount==nv)==0
           count = count+2; 
           rount(count) = nv;
           cus_next = link(1,nv);
           count = count+1;
           rount(count) = cus_next;
           while nlink(cus_next)==2
               count = count+1;
               if link(1,cus_next)==rount(count-2)
                   rount(count) = link(2,cus_next);
                   cus_next = link(2,cus_next);
               else
                   rount(count) = link(1,cus_next);
                   cus_next = link(1,cus_next);
               end
           end
        end
    end
    rount(count+1:end) = [];
end


% Roulette wheel selection
function idx = roulette(prob, k)
    if nargin < 2, k = 1; end
    prob = prob(:);
    prob(prob<0) = 0;
    s = sum(prob);
    if s <= 0
        idx = randi(numel(prob), 1, k);
        return;
    end
    prob = prob / s;
    cdf = cumsum(prob);
    r = rand(1,k);
    idx = arrayfun(@(x) find(cdf>=x,1,'first'), r);
end
