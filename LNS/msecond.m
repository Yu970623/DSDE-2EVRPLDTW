function [equip_cost2, energy_cost2, c_weight, c_energy, end_sat, E_route] = msecond(vrp2e, route, varargin)
% msecond  Evaluate 2nd-echelon (drone) routes in ALNS with hover(wait) energy.
%
% USAGE:
%   [equip, energy, cW, cE] = msecond(vrp2e, route)
%   [equip, energy, cW, cE] = msecond(vrp2e, route, seq_sat, data)
%
% route encoding (same as your ALNS):
%   route is a 1¡ÁL sequence, containing satellites (<=num_sat) as separators,
%   and customers (>num_sat) as visited nodes. Each sub-route starts with a satellite.
%   The route does NOT need to explicitly end at a satellite: this function will
%   automatically close each sub-route by returning the last customer to its nearest satellite.
%
% Hover(wait) energy rule:
%   - For each sub-route, the first customer uses "delayed departure" so that
%     the drone arrives at max(arrival, TW_start) WITHOUT waiting => wait=0.
%   - For subsequent customers, if early arrival happens, wait is counted as hover time
%     and is added to the previous flight segment energy.
%
% Time window sources (optional):
%   Prefer vrp2e.Windows (num_cus¡Á2), otherwise vrp2e.tw_start/tw_end (num_cus¡Á1).
%   If none exists, wait=0 and no TW feasibility check is performed here.
%
% Outputs:
%   equip_cost2  equipment cost (same coefficient as your old msecond_ALNS)
%   energy_cost2 energy cost (scaled as old code: 0.001 * total_energy)
%   c_weight     capacity violation (max load - capacity, >=0)
%   c_energy     endurance violation (max route energy - E, >=0)
%   end_sat      1¡Ánroutes end satellite index for each sub-route (nearest sat)
%   E_route      1¡Ánroutes energy (Wh) for each sub-route

    num_sat = vrp2e.num_sat;
    V       = vrp2e.V;             % km/h
    dis_sc  = vrp2e.dis_sc;        % km
    demand  = vrp2e.demand(:);
    Emax    = vrp2e.E;
    capMax  = vrp2e.fleet(2,1);

    % time windows (optional)
    [tw_s, tw_e, hasTW] = get_tw(vrp2e);
    service = [];
    if isfield(vrp2e,'service') && ~isempty(vrp2e.service)
        service = vrp2e.service(:);
    end

    % equipment cost coefficient (keep your old value)
    equip_coef = 3.36;
    energy_coef = 0.001;

    % Parse sub-routes by satellite separators
    route = route(:)';
    loc = find(route <= num_sat);
    if isempty(loc)
        % no satellite -> invalid encoding, but do not crash
        equip_cost2 = 0; energy_cost2 = 0; c_weight = 0; c_energy = 0;
        end_sat = []; E_route = [];
        return;
    end

    nr = numel(loc);
    end_sat = zeros(1,nr);
    E_route = zeros(1,nr);
    maxLoadAll = 0;

    for r = 1:nr
        st = loc(r);
        if r < nr, ed = loc(r+1)-1; else, ed = numel(route); end
        if ed <= st
            % empty route -> ignore
            continue;
        end
        sub = route(st:ed);

        % remove possible trailing satellite (keep open form)
        if sub(end) <= num_sat
            sub = sub(1:end-1);
            if numel(sub) < 2, continue; end
        end

        s0 = sub(1);
        cus = sub(2:end);

        % choose nearest end satellite (for closure)
        last = cus(end);
        [s_end, d_back] = nearest_sat(dis_sc, last, num_sat);
        end_sat(r) = s_end;

        % Build closed route for energy evaluation
        sub_closed = [sub, s_end];

        % Compute load profile per leg (length = numel(sub_closed)-1)
        [load_leg, maxLoad] = compute_load_profile(vrp2e, sub_closed, demand, num_sat);
        maxLoadAll = max(maxLoadAll, maxLoad);

        % Compute waiting minutes for each arrival node (except the start satellite)
        wait_min = zeros(1, numel(sub_closed)); % wait at node j (minutes), j=2..end
        if hasTW
            wait_min = compute_wait_profile(sub_closed, num_sat, dis_sc, V, tw_s, tw_e, service);
        end

        % Energy accumulation per leg, add wait at the ARRIVAL node to the PREVIOUS leg time
        E = 0;
        for k = 1:(numel(sub_closed)-1)
            i = sub_closed(k);
            j = sub_closed(k+1);
            d = dis_sc(i,j);
            t_h = d / max(1e-12, V);

            % add hover time before serving node j (if j is a customer and early)
            t_h = t_h + wait_min(k+1)/60;

            P = power_hover(vrp2e, load_leg(k));   % W (same model as old code)
            E = E + P * t_h;                       % Wh (W*h)
        end
        E_route(r) = E;
    end

    num_fleet = sum(route <= num_sat);
    equip_cost2  = num_fleet * equip_coef;
    energy_cost2 = sum(E_route) * energy_coef;

    c_weight = max(0, maxLoadAll - capMax);
    c_energy = max(0, max(E_route) - Emax);

    % Optional: if caller provides (seq_sat, data), you can refine start time constraints.
    % Here we keep the "first customer wait=0 by delaying departure" rule, so truck arrival
    % timing will not introduce hover at the first customer anyway. (Can be extended later.)
end

% ---------------- helper: time windows ----------------
function [tw_s, tw_e, hasTW] = get_tw(vrp2e)
    hasTW = false; tw_s = []; tw_e = [];
    if isfield(vrp2e,'Windows') && ~isempty(vrp2e.Windows) && size(vrp2e.Windows,2) >= 2
        tw_s = vrp2e.Windows(:,1);
        tw_e = vrp2e.Windows(:,2);
        hasTW = true;
        return;
    end
    if isfield(vrp2e,'tw_start') && isfield(vrp2e,'tw_end') && ~isempty(vrp2e.tw_start)
        tw_s = vrp2e.tw_start(:);
        tw_e = vrp2e.tw_end(:);
        hasTW = true;
    end
end

function smin = serv_time(node, num_sat, service)
    if isempty(service), smin = 0; return; end
    if numel(service) >= node
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

% ---------------- helper: nearest satellite ----------------
function [s_end, dmin] = nearest_sat(dis_sc, node, num_sat)
    dmin = inf; s_end = 1;
    for s = 1:num_sat
        if dis_sc(node,s) < dmin
            dmin = dis_sc(node,s);
            s_end = s;
        end
    end
end

% ---------------- helper: load profile ----------------
function [load_leg, maxLoad] = compute_load_profile(vrp2e, sub_closed, demand, num_sat)
    % sub_closed: [sat, cus..., sat_end]
    cus_type = [];
    if isfield(vrp2e,'type') && ~isempty(vrp2e.type)
        cus_type = vrp2e.type;
    end

    % identify customers (exclude satellites)
    nodes = sub_closed(:)';
    cus = nodes(nodes > num_sat & nodes <= num_sat + vrp2e.num_cus);

    % initial load: sum of DELIVERY demands in this sub-route
    init_load = 0;
    for k = 1:numel(cus)
        c = cus(k);
        if isempty(cus_type)
            init_load = init_load + demand(c); % all as delivery
        else
            tp = cus_type{c-num_sat};
            if ~strcmpi(tp,'pickup')
                init_load = init_load + demand(c);
            end
        end
    end

    load_leg = zeros(1, numel(nodes)-1);
    load = init_load;
    maxLoad = load;

    for k = 1:(numel(nodes)-1)
        i = nodes(k);
        j = nodes(k+1);

        load_leg(k) = load;
        maxLoad = max(maxLoad, load);

        % update load AFTER arriving at j (if j is customer)
        if j > num_sat
            if isempty(cus_type)
                load = load - demand(j);
            else
                tp = cus_type{j-num_sat};
                if strcmpi(tp,'pickup')
                    load = load + demand(j);
                else
                    load = load - demand(j);
                end
            end
        else
            % satellite: route ends, load irrelevant
        end
    end
end

% ---------------- helper: wait profile (minutes) ----------------
function wait_min = compute_wait_profile(sub_closed, num_sat, dis_sc, V, tw_s, tw_e, service)
    % wait_min(j) is hover waiting at node j (minutes), j indexes sub_closed.
    % j=1 is the start satellite => wait_min(1)=0.
    nodes = sub_closed(:)';
    wait_min = zeros(1, numel(nodes));

    % Find first customer in this sub-route
    if numel(nodes) < 3
        return;
    end

    sat = nodes(1);
    first = nodes(2);
    if first <= num_sat
        return;
    end

    travel_min = @(i,j) dis_sc(i,j) / max(1e-12, V) * 60;

    % ---- delayed departure for first customer: set start time so that first has no wait
    idx0 = first - num_sat;
    t0_arr = serv_time(sat, num_sat, service) + travel_min(sat, first);

    if t0_arr > tw_e(idx0) + 1e-9
        % already too late: no waiting, keep baseline 0
        t = t0_arr;
    else
        % choose departure time so arrival >= tw_start
        t = max(t0_arr, tw_s(idx0));  % time at first customer after possible delay
    end
    % by construction wait_min(first)=0 (do not count hover before first)
    prev = first;

    % ---- subsequent customers
    for k = 3:(numel(nodes)-1) % last node is satellite end
        cur = nodes(k);
        if cur <= num_sat
            break;
        end
        t_arr = t + serv_time(prev, num_sat, service) + travel_min(prev, cur);

        idx = cur - num_sat;
        if t_arr > tw_e(idx) + 1e-9
            % too late: no waiting (still 0), but leave it
            wait_min(k) = 0;
            t = t_arr;
        else
            if t_arr < tw_s(idx)
                wait_min(k) = tw_s(idx) - t_arr; % hover wait before serving cur
                t = tw_s(idx);
            else
                wait_min(k) = 0;
                t = t_arr;
            end
        end
        prev = cur;
    end
end

% ---------------- helper: power model ----------------
function P = power_hover(vrp2e, load)
    % Same power model as your old code (constant C)
    C = (9.81^3) / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);
    P = (5 + load).^(3/2) * C;
end
