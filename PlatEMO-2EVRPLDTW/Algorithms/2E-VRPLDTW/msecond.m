function [equip_cost2, energy_cost2, c_weight, c_energy] = msecond(vrp2e, route2, seq_sat, data)
% MSECOND  Second-echelon evaluation (energy) aligned with insert_cus + time_violate.
%
% Key alignment points:
%   1) Flight time for time-window/waiting uses data.T2 (same as time_violate),
%      fallback to dis_sc/V*60 if T2 absent.
%   2) First customer: delayed takeoff -> hover(wait) energy = 0.
%   3) sat_ready for each subroute should be: time_first_at_satellite_position + service(sat)
%      (i.e., time_second at satellite in time_violate). Then departure lower bound is sat_ready (truck arrival + one service at satellite).
%
% Outputs keep the original 4-output signature.
% This function does NOT turn TW violations into inf anymore; TW feasibility is expected to be handled
% by insert_cus/time_violate separately. (Energy is still computed; c_energy captures battery violation.)
%

if nargin < 2
    error('msecond: need at least (vrp2e, route2).');
end
if nargin < 3, seq_sat = []; end
if nargin < 4, data = []; end

num_sat = vrp2e.num_sat;
fleet   = vrp2e.fleet;
Emax    = vrp2e.E;

route = route2(route2>0);
if isempty(route)
    equip_cost2 = 0;
    energy_cost2 = 0;
    c_weight = 0;
    c_energy = 0;
    return;
end

% service/windows
[service, windows] = get_service_windows(data, vrp2e);

% Build sat_ready per satellite from seq_sat (optional)
sat_ready_map = zeros(1, num_sat);
if ~isempty(seq_sat) && ~isempty(data)
    sat_ready_map = compute_sat_ready_map(seq_sat, data, vrp2e, service);
end

% Split into subroutes
sat_pos = find(route <= num_sat);
sat_pos = sat_pos(:)';

Energy_sat = zeros(1, numel(sat_pos));
c_energy = 0;
c_weight = 0;

% power constant
Cc = (9.81^3) / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);

for r = 1:numel(sat_pos)
    sidx = sat_pos(r);
    eidx = numel(route);
    if r < numel(sat_pos)
        eidx = sat_pos(r+1) - 1;
    end

    nodes = route(sidx:eidx); % [sat, c1, c2, ...]
    sat = nodes(1);
    cus = nodes(nodes > num_sat);
    if isempty(cus)
        Energy_sat(r) = 0;
        continue;
    end

    % nearest return satellite for energy
    last = cus(end);
    [~, s_end] = min(vrp2e.dis_sc(last, 1:num_sat));

    % payload profile
    try
        cap = cap_realtime(vrp2e, [nodes, s_end]);
    catch
        cap = zeros(1, numel(nodes));
    end

    % sat_ready should already include one service(sat) to match time_violate
    sat_ready = 0;
    if sat <= numel(sat_ready_map)
        sat_ready = sat_ready_map(sat);
    end

    srv_sat = 0;
    if numel(service) >= sat, srv_sat = service(sat); end

    % ---------- first customer (no hover energy) ----------
    c1 = cus(1);
    fly1 = fly_time_min(sat, c1, num_sat, data, vrp2e);
    e1 = windows(c1-num_sat,1);
    % delayed takeoff:
    t_dep = max(sat_ready, e1 - fly1);
    t_arr = t_dep + fly1;
    t_srv = max(t_arr, e1);
    if numel(service) >= c1, t_srv = t_srv + service(c1); end

    E = segment_energy(cap, 1, Cc, fly1/60, 0);
    prev = c1;
    t_prev = t_srv;

    % ---------- subsequent customers ----------
    for k = 2:numel(cus)
        cur = cus(k);
        fly = fly_time_min(prev, cur, num_sat, data, vrp2e);

        t_arr = t_prev + fly;

        e = windows(cur-num_sat,1);
        wait = max(0, e - t_arr);

        t_srv = t_arr + wait;
        if numel(service) >= cur, t_srv = t_srv + service(cur); end

        E = E + segment_energy(cap, k, Cc, fly/60, wait/60);

        prev = cur;
        t_prev = t_srv;
    end

    % ---------- return energy ----------
    fly_back = vrp2e.dis_sc(prev, s_end) / max(1e-9, vrp2e.V) * 60; % minutes
    E = E + segment_energy(cap, numel(cus)+1, Cc, fly_back/60, 0);

    Energy_sat(r) = E;

    if E > Emax + 1e-9
        c_energy = c_energy + (E - Emax);
    end
end

% equipment + energy costs (keep your original style)
% equipment cost: number of drone tours beyond fleet(2,2)
num_fleet = numel(sat_pos);

% If your model uses a fixed equipment cost per drone tour, put it here.
% We keep "equip_cost2 = c_fleet" as a placeholder; adjust if you have a coefficient.
equip_cost2 = num_fleet*3.36;

energy_cost2 = sum(Energy_sat)*0.001;

end

% ======================================================================
% Helper functions
% ======================================================================

function Eseg = segment_energy(cap, idx, Cc, fly_h, wait_h)
    if idx > numel(cap)
        load = cap(end);
    else
        load = cap(idx);
    end
    P = (5 + load).^(3/2) * Cc;
    Eseg = P * (fly_h + wait_h);
end

function [service, windows] = get_service_windows(data, vrp2e)
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

function fly = fly_time_min(from_node, to_node, num_sat, data, vrp2e)
    if isstruct(data) && isfield(data,'T2') && ~isempty(data.T2)
        try
            fly = data.T2(from_node, to_node - num_sat);
            if ~isfinite(fly), fly = inf; end
            return;
        catch
        end
    end
    fly = vrp2e.dis_sc(from_node, to_node) / max(1e-9, vrp2e.V) * 60;
end

function sat_ready_map = compute_sat_ready_map(seq_sat, data, vrp2e, service)
    % Map sat_id (1..num_sat) -> sat_ready_current = time_first(pos) + service(sat)
    num_sat = vrp2e.num_sat;
    num_dep = length(vrp2e.rad_ds(:,1));

    sat_ready_map = zeros(1, num_sat);

    % Need T1
    if ~isstruct(data) || ~isfield(data,'T1') || isempty(data.T1)
        return;
    end
    T1 = data.T1;

    time_first = zeros(1, numel(seq_sat));
    try
        for i = 2:numel(seq_sat)
            prev = seq_sat(i-1);
            cur  = seq_sat(i);

            if prev <= num_dep
                time_first(i) = time_first(i-1) + T1(prev, cur - num_dep);
            else
                sat_prev = prev - num_dep;
                time_first(i) = time_first(i-1) + service(sat_prev) + T1(sat_prev, cur - num_dep);
            end
        end
    catch
        % If T1 dimensions/indexing differ in your dataset, fall back to zeros.
        time_first(:) = 0;
    end

    for s = 1:num_sat
        node_id = s + num_dep;
        pos = find(seq_sat == node_id, 1, 'first');
        if ~isempty(pos)
            % IMPORTANT: include one service(s) here to match time_violate time_second(sat)
            sat_ready_map(s) = time_first(pos) + service(s);
        else
            sat_ready_map(s) = 0;
        end
    end
end
