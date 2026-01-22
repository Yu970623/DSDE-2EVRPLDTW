function fvlt2 = time_violate(seq_sat, seq_cus, data, vrp2e)
% TIME_VIOLATE  Second-echelon time-window violation evaluator.
% This version is aligned with instert_cus_aligned_T2_v2.m and msecond_aligned_T2.m:
%   - Flight time uses data.T2(from_node, cus_index) if available; otherwise dis_sc/V.
%   - Satellite ready time is computed from seq_sat with the SAME indexing rule as msecond_aligned_T2:
%       time_first(pos) is truck ARRIVAL time at the node in seq_sat (before service at that node).
%       For satellite s: sat_ready(s) = time_first(pos_s) + service(s).
%   -   - When flying from a satellite to the first customer, we allow *delayed takeoff* so the drone can arrive exactly at the earliest time window when the truck is early, and arrive later when the truck is late:
%       depart = max(sat_ready(s), e1 - fly_time(s->c1));
%       arrival(c1) = depart + fly_time(s->c1) = max(sat_ready(s)+fly_time, e1).
%
% Inputs:
%   seq_sat : first-echelon sequence in node-id (depot nodes: 1..num_dep, satellite nodes: num_dep+1..num_dep+num_sat)
%   seq_cus : concatenated second-echelon sequence [sat, c, c, sat, c, ...] where sat is 1..num_sat and c is (num_sat+1..)
%   data    : must contain T1, optionally T2, Services, Windows; otherwise vrp2e fields are used.
%   vrp2e   : problem struct (num_dep, num_sat, num_cus, V, dis_sc, ...)

    num_dep = size(vrp2e.rad_ds,1);
    num_sat = vrp2e.num_sat;
    num_cus = vrp2e.num_cus;

    % -------- services / windows --------
    if isfield(data,'Services') && ~isempty(data.Services)
        service = data.Services(:)'; % indexed by sat (1..num_sat) and customer node ids (num_sat+1..)
    elseif isfield(vrp2e,'service') && ~isempty(vrp2e.service)
        service = vrp2e.service(:)';
    else
        service = zeros(1, num_sat + num_cus);
    end

    if isfield(data,'Windows') && ~isempty(data.Windows)
        windows = data.Windows; % num_cus x 2, indexed by (cus_node - num_sat)
    elseif isfield(vrp2e,'Windows') && ~isempty(vrp2e.Windows)
        windows = vrp2e.Windows;
    else
        windows = [zeros(num_cus,1), inf(num_cus,1)];
    end

    % -------- T1 / T2 --------
    T1 = [];
    if isfield(data,'T1') && ~isempty(data.T1)
        T1 = data.T1;
    end
    hasT2 = isfield(data,'T2') && ~isempty(data.T2);

    % -------- compute truck arrival times along seq_sat --------
    % time_first(i): arrival time at seq_sat(i) BEFORE service at that node.
    % If data already provides a compatible time_first, you can use it.
    time_first = zeros(1, numel(seq_sat));
    if isfield(data,'time_first') && ~isempty(data.time_first) && numel(data.time_first) == numel(seq_sat)
        time_first = data.time_first(:)'; % trust upstream (optional)
    elseif ~isempty(T1) && numel(seq_sat) >= 2
        try
            for i = 2:numel(seq_sat)
                prev = seq_sat(i-1);
                cur  = seq_sat(i);

                if prev <= num_dep
                    % depot -> satellite node
                    time_first(i) = time_first(i-1) + T1(prev, cur - num_dep);
                else
                    % satellite -> satellite : IMPORTANT alignment with msecond_aligned_T2
                    sat_prev = prev - num_dep;
                    time_first(i) = time_first(i-1) + service(sat_prev) + T1(sat_prev, cur - num_dep);
                end
            end
        catch
            time_first(:) = 0;
        end
    end

    % -------- satellite ready map: sat_ready(s) = arrival_at_sat + service(s) --------
    sat_ready = zeros(1, num_sat);
    for s = 1:num_sat
        node_id = num_dep + s;
        pos = find(seq_sat == node_id, 1, 'first');
        if ~isempty(pos)
            sat_ready(s) = time_first(pos) + service(s);
        else
            sat_ready(s) = 0;
        end
    end

    % -------- evaluate second-echelon sequence --------
    fvlt2 = 0;
    if isempty(seq_cus)
        return;
    end

    sat_pos = find(seq_cus <= num_sat);
    if isempty(sat_pos)
        return;
    end

    for r = 1:numel(sat_pos)
        sidx = sat_pos(r);
        eidx = numel(seq_cus);
        if r < numel(sat_pos)
            eidx = sat_pos(r+1) - 1;
        end

        nodes = seq_cus(sidx:eidx);  % [sat, c1, c2, ...]
        sat = nodes(1);
        cus = nodes(nodes > num_sat);
        if isempty(cus)
            continue;
        end

        % time at satellite after ONE service (sat_ready already includes 1 service)
        t_prev = sat_ready(sat);

        % ------- first customer: delayed takeoff (no hover waiting at c1) -------
        c1 = cus(1);
        fly1 = fly_time_min(sat, c1, num_sat, vrp2e, data, hasT2);

        e1 = windows(c1 - num_sat, 1);
        l1 = windows(c1 - num_sat, 2);

        % depart >= sat_ready(sat); if truck is early, delay takeoff so arrival hits e1
        t_arr = max(t_prev + fly1, e1); % minutes

        if t_arr > l1
            fvlt2 = fvlt2 + (t_arr - l1);
        end

        if numel(service) >= c1
            t_prev = t_arr + service(c1);
        else
            t_prev = t_arr;
        end

        % ------- subsequent customers -------
        for k = 2:numel(cus)
            cur = cus(k);
            fly = fly_time_min(cus(k-1), cur, num_sat, vrp2e, data, hasT2);
            t_arr = t_prev + fly;

            e = windows(cur - num_sat, 1);
            l = windows(cur - num_sat, 2);

            if t_arr > l
                fvlt2 = fvlt2 + (t_arr - l);
            end

            t_start = max(t_arr, e);
            if numel(service) >= cur
                t_prev = t_start + service(cur);
            else
                t_prev = t_start;
            end
        end
    end
end

function fly = fly_time_min(from_node, to_node, num_sat, vrp2e, data, hasT2)
% Return flight time in minutes from from_node (sat or customer node id) to to_node (customer node id)
    if hasT2
        try
            fly = data.T2(from_node, to_node - num_sat);
            if isfinite(fly)
                return;
            end
        catch
        end
    end
    fly = vrp2e.dis_sc(from_node, to_node) / max(1e-9, vrp2e.V) * 60;
end
