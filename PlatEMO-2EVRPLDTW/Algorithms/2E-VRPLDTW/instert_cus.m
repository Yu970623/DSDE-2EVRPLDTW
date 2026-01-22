function [route_new, min_cost, Remain_battery, need_newroute2] = instert_cus(varargin)
% INSTERT_CUS  Insert one customer into a 2nd-echelon drone subroute.
%
% IMPORTANT: Interface must remain unchanged to avoid upstream errors.
% Supported calling conventions (auto-detected):
%   1) [r,~,~,need] = instert_cus(data, vrp2e, route, c)
%   2) [r,need,remain] = instert_cus(vrp2e, route, c, data)  (legacy)
%
% Inputs
%   route : a single subroute [sat, cus, cus, ...] (NO closing satellite)
%   c     : a global node id of a customer (num_sat+1 ... num_sat+num_cus)
%
% Outputs
%   route_new      updated subroute (same format)
%   min_cost       incremental cost used for selection (smaller is better)
%   Remain_battery remaining ratio in [0,1] for this subroute (after insertion)
%   need_newroute2 1 if cannot insert into this subroute (recommend opening new)
%
% Design goals (aligning with Gurobi patterns):
%   - Prefer inserting into existing subroutes/satellites;
%   - When opening a new subroute, strongly prefer already-used satellites;
%   - Keep feasibility checks for payload, time window, and energy (incl. hover);
%   - Keep computation lightweight (satellites/customers are single-digit).

% -------------------- argument parsing --------------------
[data, vrp2e, route, c] = parse_args(varargin{:});
num_sat = vrp2e.num_sat;

route = route(route > 0);
need_newroute2 = 0;
Remain_battery = -1;
min_cost = inf;

% If caller provides an empty route, we directly open a new subroute.
if isempty(route)
    [route_new, min_cost, Remain_battery] = open_new_subroute(data, vrp2e, c);
    need_newroute2 = 0;
    return;
end
% Ensure the route is a single subroute starting with a satellite marker.
if route(1) > num_sat
    % If a pure customer sequence is passed, attach a satellite by a robust rule.
    s0 = nearest_sat(vrp2e.dis_sc, num_sat, route(1));
    route = [s0, route(:)'];
else
    route = route(:)';
end

% Defensive: if route contains multiple satellite markers, keep ONLY the first subroute.
% (Global insertion across multiple subroutes should be handled in aimFcn10.)
sat_pos = find(route <= num_sat);
if numel(sat_pos) > 1
    route = route(1:sat_pos(2)-1);
end

% Evaluate current route cost once (feasibility should already hold).
[ok0, E0, ~] = eval_subroute(route, vrp2e, data);
if ~ok0
    % If current route is infeasible, we still attempt insertion by rebuilding;
    % but we set baseline to 0 so min_cost becomes absolute.
    E0 = 0;
end

best_route = [];
best_inc = inf;
best_rem = -1;

% Candidate insertion positions: insert customer after any existing node.
% (position 2 means right after the satellite marker)
for pos = 2:(numel(route)+1)
    cand = [route(1:pos-1), c, route(pos:end)];
    [ok, Enew, remain] = eval_subroute(cand, vrp2e, data);
    if ~ok
        continue;
    end

    inc = Enew - E0;
    % Tie-break: prefer larger remaining battery (less tight routes)
    inc = inc - 1e-6 * remain;

    if inc < best_inc
        best_inc = inc;
        best_route = cand;
        best_rem = remain;
    end
end

if isempty(best_route)
    % Cannot insert into this subroute.
    route_new = route;
    min_cost = inf;
    Remain_battery = -1;
    need_newroute2 = 1;
else
    route_new = best_route;
    min_cost = best_inc;
    Remain_battery = best_rem;
    need_newroute2 = 0;
end

end

% ======================================================================
% Helpers
% ======================================================================

function [data, vrp2e, route, c] = parse_args(varargin)
    % Auto-detect the two supported signatures.
    if nargin == 4 && isstruct(varargin{1}) && isfield(varargin{1}, 'vrp2e')
        data = varargin{1};
        vrp2e = varargin{2};
        route = varargin{3};
        c = varargin{4};
        return;
    end
    if nargin == 4 && isstruct(varargin{1}) && isfield(varargin{1}, 'num_sat')
        vrp2e = varargin{1};
        route = varargin{2};
        c = varargin{3};
        data = varargin{4};
        return;
    end
    error('instert_cus: invalid inputs. Expected (data,vrp2e,route,c) or (vrp2e,route,c,data).');
end

function [route_new, score, remain_ratio] = open_new_subroute(data, vrp2e, c)
    % Open a new subroute [sat, c] with strong preference for already-used satellites.
    num_sat = vrp2e.num_sat;
    Emax = vrp2e.E;

    used_sats = [];
    if isfield(data, 'used_sats') && ~isempty(data.used_sats)
        used_sats = unique(data.used_sats(:)');
    end

    % Satellite forbidden list (optional)
    s_off = [];
    if isfield(data,'s_off') && ~isempty(data.s_off)
        s_off = data.s_off(:)';
    end

    % Penalties (heuristic, to imitate Gurobi's "use fewer sats/routes")
    [pen_new_route, pen_new_sat, alpha_central] = get_penalties(vrp2e);

    % When no satellite has been used yet, the *first* satellite choice is critical.
    % We add a light "coverage" + stronger centrality guidance to favor a globally good hub satellite.
    used_empty = isempty(used_sats);
    if used_empty
        alpha_central = max(alpha_central, 0.02);   % slightly stronger hub bias for the first route
        beta_cov = 0.12;                            % reward satellites that can feasibly serve many remaining customers
    else
        beta_cov = 0.05;
    end

    % Optional centrality guidance: prefer satellites close to many remaining customers.
    remain_nodes = [];
    central_pen = zeros(1, num_sat);
    if alpha_central > 0 && isfield(data,'remain_cus') && ~isempty(data.remain_cus)
        remain_nodes = unique(data.remain_cus(:)');
        remain_nodes = remain_nodes(remain_nodes > vrp2e.num_sat);
        if ~isempty(remain_nodes) && isfield(vrp2e,'dis_sc') && ~isempty(vrp2e.dis_sc)
            for s = 1:num_sat
                try
                    central_pen(s) = mean(vrp2e.dis_sc(remain_nodes, s));
                catch
                    central_pen(s) = 0;
                end
            end
        end
    end

    % Coverage list: include current customer and remaining ones (cap length to keep it fast).
    cov_nodes = [c, remain_nodes];
    cov_nodes = unique(cov_nodes);
    cov_nodes = cov_nodes(cov_nodes > vrp2e.num_sat);
    if numel(cov_nodes) > 12
        cov_nodes = cov_nodes(1:12);
    end

    bestScore = inf;
    bestS = 1;
    bestE = inf;
    bestRemain = -1;

    % Two-phase try: (1) used sats, (2) all sats
    candidate_sets = {used_sats, 1:num_sat};
    for phase = 1:2
        candS = candidate_sets{phase};
        if isempty(candS)
            continue;
        end
        for s = candS
            if ~isempty(s_off) && any(s_off == s)
                continue;
            end
            sub = [s, c];
            [ok, Esub, remain] = eval_subroute(sub, vrp2e, data);
            if ~ok || ~isfinite(Esub) || Esub > Emax + 1e-9
                continue;
            end

            isNewSat = isempty(used_sats) || ~any(used_sats == s);

            % Coverage reward: satellites that can (individually) serve more remaining customers
            % tend to become a good "hub" satellite, reducing satellite count in the final solution.
            cov_cnt = 0;
            if beta_cov > 0 && ~isempty(cov_nodes)
                cov_cnt = cov_feasible_count(s, cov_nodes, vrp2e, data);
            end

            score = Esub + pen_new_route + pen_new_sat * double(isNewSat) ...
                    + alpha_central * central_pen(s) - beta_cov * cov_cnt;

            % Tie-break: prefer smaller energy then larger remaining
            score = score - 1e-6 * remain;

            if score < bestScore
                bestScore = score;
                bestS = s;
                bestE = Esub;
                bestRemain = remain;
            end
        end

        % If we found a feasible satellite among used ones, stop early.
        if phase == 1 && isfinite(bestScore)
            break;
        end
    end

    route_new = [bestS, c];
    score = bestScore;
    remain_ratio = bestRemain;
end

function [ok, Esub, remain_ratio] = eval_subroute(subroute, vrp2e, data)
    % Feasibility + energy evaluation for one subroute [sat, c1, c2, ...]
    ok = true;
    Esub = 0;
    remain_ratio = 0;

    num_sat = vrp2e.num_sat;
    Emax = vrp2e.E;

    if isempty(subroute) || subroute(1) > num_sat
        ok = false; Esub = inf; remain_ratio = 0; return;
    end

    sat = subroute(1);
    cus = subroute(subroute > num_sat);
    if isempty(cus)
        ok = true; Esub = 0; remain_ratio = 1; return;
    end

    % Retrieve service + time windows
    [service, windows] = get_service_windows(data, vrp2e);

    % Drone capacity (fallback)
    droneCap = inf;
    if isfield(vrp2e,'fleet') && size(vrp2e.fleet,1) >= 2
        droneCap = vrp2e.fleet(2,1);
    end

    % Build payload profile (exact for delivery/pickup if type exists)
    [load_profile, maxLoad] = payload_profile(vrp2e, cus);
    if maxLoad > droneCap + 1e-9
        ok = false; Esub = inf; remain_ratio = 0; return;
    end

    % Constant in power model (keep consistent with your earlier implementation)
    Cc = (9.81^3) / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

    % Start time at satellite (if provided)
    sat_ready = 0;
    if isfield(data,'sat_ready_current') && ~isempty(data.sat_ready_current)
        sat_ready = data.sat_ready_current;
    end

    srv_sat = 0;
    if numel(service) >= sat
        srv_sat = service(sat);
    end

    % ---------- first customer: delayed takeoff, no hover energy ----------
    c1 = cus(1);
    fly1 = fly_time_min(sat, c1, num_sat, data, vrp2e);
    if ~isfinite(fly1)
        ok = false; Esub = inf; remain_ratio = 0; return;
    end

    [e1, l1] = win_of(windows, c1-num_sat);
    t_dep = max(sat_ready, e1 - fly1);
    t_arr = t_dep + fly1;
    if t_arr > l1 + 1e-9
        ok = false; Esub = inf; remain_ratio = 0; return;
    end

    % Arrival service time
    t_srv = max(t_arr, e1);
    if numel(service) >= c1
        t_srv = t_srv + service(c1);
    end

    % Energy sat->c1 (hover=0 by delayed takeoff rule)
    Esub = Esub + seg_energy(load_profile(1), Cc, fly1/60, 0);
    if Esub > Emax + 1e-9
        ok = false; Esub = inf; remain_ratio = 0; return;
    end

    prev = c1;
    t_prev = t_srv;

    % ---------- subsequent customers (hover energy counts) ----------
    for k = 2:numel(cus)
        cur = cus(k);
        fly = fly_time_min(prev, cur, num_sat, data, vrp2e);
        if ~isfinite(fly)
            ok = false; Esub = inf; remain_ratio = 0; return;
        end

        t_arr = t_prev + fly;
        [e, l] = win_of(windows, cur-num_sat);
        if t_arr > l + 1e-9
            ok = false; Esub = inf; remain_ratio = 0; return;
        end

        wait = max(0, e - t_arr);
        t_srv = t_arr + wait;
        if numel(service) >= cur
            t_srv = t_srv + service(cur);
        end

        Esub = Esub + seg_energy(load_profile(k), Cc, fly/60, wait/60);
        if Esub > Emax + 1e-9
            ok = false; Esub = inf; remain_ratio = 0; return;
        end

        prev = cur;
        t_prev = t_srv;
    end

    % ---------- return to nearest satellite (no time window, no hover) ----------
    s_end = nearest_sat(vrp2e.dis_sc, num_sat, prev);
    fly_back = fly_back_min(prev, s_end, vrp2e, data);
    Esub = Esub + seg_energy(load_profile(end), Cc, fly_back/60, 0);

    if Esub > Emax + 1e-9
        ok = false; Esub = inf; remain_ratio = 0; return;
    end

    remain_ratio = max(0, 1 - Esub / max(Emax, 1e-9));
end

function [service, windows] = get_service_windows(data, vrp2e)
    % service: length >= (num_sat+num_cus) is preferred
    if isstruct(data) && isfield(data,'Services') && ~isempty(data.Services)
        service = data.Services(:)';
    elseif isfield(vrp2e,'service') && ~isempty(vrp2e.service)
        service = vrp2e.service(:)';
    else
        service = zeros(1, vrp2e.num_sat + vrp2e.num_cus);
    end

    if isstruct(data) && isfield(data,'Windows') && ~isempty(data.Windows)
        windows = data.Windows;
    elseif isfield(vrp2e,'windows') && ~isempty(vrp2e.windows)
        windows = vrp2e.windows;
    else
        windows = [zeros(vrp2e.num_cus,1), inf(vrp2e.num_cus,1)];
    end
end

function [e, l] = win_of(windows, idx)
    if idx < 1 || idx > size(windows,1)
        e = 0; l = inf;
    else
        e = windows(idx,1);
        l = windows(idx,2);
    end
end

function fly = fly_time_min(from_node, to_node, num_sat, data, vrp2e)
    % Flight time (minutes). Prefer precomputed T2.
    fly = inf;
    if isstruct(data) && isfield(data,'T2') && ~isempty(data.T2)
        try
            fly = data.T2(from_node, to_node - num_sat);
            return;
        catch
            % fallback
        end
    end
    % fallback by distance and speed
    try
        fly = vrp2e.dis_sc(from_node, to_node) / max(1e-9, vrp2e.V) * 60;
    catch
        fly = inf;
    end
end

function fly = fly_back_min(from_node, sat_to, vrp2e, data)
    %#ok<INUSD>
    % Return flight time (minutes) from last customer to a satellite.
    fly = inf;
    try
        fly = vrp2e.dis_sc(from_node, sat_to) / max(1e-9, vrp2e.V) * 60;
        if isfinite(fly)
            return;
        end
    catch
        % fallback
    end
    fly = inf;
end

function E = seg_energy(load, Cc, fly_h, hover_h)
    % Energy = Power(load) * (flight time + hover time)
    P = (5 + load).^(3/2) * Cc;
    E = P * (fly_h + hover_h);
end

function sat = nearest_sat(dis_sc, num_sat, node)
    % Find nearest satellite for a node (customer) based on distance matrix.
    try
        [~, sat] = min(dis_sc(node, 1:num_sat));
    catch
        sat = 1;
    end
end

function [pen_new_route, pen_new_sat, alpha_central] = get_penalties(vrp2e)
    % Heuristic penalties (lightweight):
    % - pen_new_route: discourages opening new drone subroutes (reduces #drones => obj1)
    % - pen_new_sat  : discourages activating new satellites (reduces first-echelon burden)
    % - alpha_central: soft guidance to choose a "central" satellite when starting routes

    pen_new_route = 0;
    pen_new_sat = 0;
    alpha_central = 0;

    if isfield(vrp2e,'fleet') && ~isempty(vrp2e.fleet)
        try
            % fleet(2,2) is commonly used as a fixed drone cost in your codebase.
            pen_new_route = max(0, vrp2e.fleet(2,2));
        catch
            pen_new_route = 0;
        end
        try
            % fleet(1,2) commonly denotes a fixed truck cost; satellite activation impacts trucks.
            pen_new_sat = 0.5 * max(0, vrp2e.fleet(1,2));
        catch
            pen_new_sat = 0;
        end
    end

    % If fixed costs are unavailable, still set gentle penalties.
    if pen_new_route == 0
        pen_new_route = 1;
    end
    if pen_new_sat == 0
        pen_new_sat = 3 * pen_new_route;
    end

    % Centrality weight: small number to avoid overriding feasibility/energy.
    alpha_central = 0.01;
end

function [load_prof, maxLoad] = payload_profile(vrp2e, cus)
    % Compute load before each leg, with delivery/pickup handling if available.
    % load_prof length = nCus + 1 (includes return leg).
    n = numel(cus);
    load_prof = zeros(1, n+1);

    demand = [];
    if isfield(vrp2e,'demand') && ~isempty(vrp2e.demand)
        demand = vrp2e.demand(:)';
    end

    % Default: treat all as delivery (loaded at start).
    isDelivery = true(1,n);
    if isfield(vrp2e,'type') && ~isempty(vrp2e.type)
        try
            type = vrp2e.type;
            for i = 1:n
                idx = cus(i) - vrp2e.num_sat;
                if idx >= 1 && idx <= numel(type)
                    isDelivery(i) = strcmp(type{idx}, 'delivery');
                end
            end
        catch
            % keep default
        end
    end

    dem = zeros(1,n);
    for i = 1:n
        if ~isempty(demand) && cus(i) <= numel(demand)
            dem(i) = demand(cus(i));
        else
            dem(i) = 0;
        end
    end

    % Initial load = sum of deliveries.
    load = sum(dem(isDelivery));
    maxLoad = load;
    load_prof(1) = load;

    for k = 1:n
        if isDelivery(k)
            load = load - dem(k);
        else
            load = load + dem(k);
        end
        maxLoad = max(maxLoad, load);
        if k < n
            load_prof(k+1) = load;
        else
            load_prof(n+1) = load; % for return leg
        end
    end

    % Ensure non-negative
    load_prof(load_prof < 0) = 0;
end

function cnt = cov_count_single(s, cov_nodes, vrp2e, data)
% COV_COUNT_SINGLE Count how many customers are individually feasible from satellite s.
% This is a weak heuristic hint used in open_new_subroute when choosing the
% initial satellite (when no used_sats exist yet).

    cnt = 0;
    if isempty(cov_nodes)
        return;
    end

    % Downsample to keep runtime low (we only need a rough signal)
    maxCheck = min(12, numel(cov_nodes));
    nodes = cov_nodes(:)';
    if isfield(vrp2e,'dis_sc') && ~isempty(vrp2e.dis_sc)
        try
            d = vrp2e.dis_sc(s, nodes);
            [~, ord] = sort(d, 'ascend');
            nodes = nodes(ord);
        catch
            % keep original order
        end
    end
    nodes = nodes(1:maxCheck);

    % Use the exact feasibility evaluator on single-customer sorties.
    for ii = 1:numel(nodes)
        sub = [s, nodes(ii)];
        [ok, ~, ~] = eval_subroute(sub, vrp2e, data);
        if ok
            cnt = cnt + 1;
        end
    end
end

function cnt = cov_feasible_count(s, cov_nodes, vrp2e, data)
% COV_FEASIBLE_COUNT Backward-compatible wrapper.
% Earlier revisions referenced this name; the implementation is identical
% to cov_count_single.
    cnt = cov_count_single(s, cov_nodes, vrp2e, data);
end
