function cap_sc = cap_realtime(vrp2e,route)
%CAP_REALTIME 计算二级无人机路径各段“实时载重”（用于功率/能耗计算）
%
% 输入：
%   vrp2e.demand : 节点需求（卫星为0，客户节点为需求量）
%   vrp2e.type   : 客户类型 cellstr，'pickup' 或 'delivery'
%   route        : 串联的二级路径（卫星+客户）；通常末尾会带一个哨兵，因此这里默认丢掉最后一个元素
%
% 输出：
%   cap_sc(i) : 从 route(i) 出发沿 route(i)->next 的那一段，离开 route(i) 时无人机载重
%
    demand  = vrp2e.demand(:);
    num_sat = vrp2e.num_sat;

    % 兼容：若末尾为哨兵，去掉最后一个
    if numel(route) >= 2
        route = route(1:end-1);
    end
    if isempty(route)
        cap_sc = [];
        return;
    end

    cap_s   = zeros(1,numel(route));
    loc_sat = find(route<=num_sat);

    for i = 1:numel(route)
        node = route(i);
        if node <= num_sat
            cap_s(i) = 0;
            continue;
        end

        prev_cap = 0;
        if i > 1
            prev_cap = cap_s(i-1);
        end

        is_pickup = false;
        if isfield(vrp2e,'type') && ~isempty(vrp2e.type)
            try
                is_pickup = strcmpi(vrp2e.type{node-num_sat},'pickup') || strcmpi(vrp2e.type(node-num_sat),'pickup');
            catch
                is_pickup = false;
            end
        end

        if is_pickup
            cap_s(i) = prev_cap + demand(node);
        else
            cap_s(i) = prev_cap;
            last_sat_pos = find(loc_sat < i, 1, 'last');
            if ~isempty(last_sat_pos)
                s_pos = loc_sat(last_sat_pos);
                if s_pos <= i-1
                    cap_s(s_pos:i-1) = cap_s(s_pos:i-1) + demand(node);
                end
            end
        end
    end

    cap_sc = cap_s;
end
