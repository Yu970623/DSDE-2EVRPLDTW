function [seq_sat,cap_sat] = insert_sat(vrp2e,seq_r)
%INSERT_SAT  Fast and small-scale exact first-echelon construction.
%
% This constructor is designed to mimic the patterns of Gurobi-optimal solutions:
%   (i)  Prefer using as FEW first-echelon routes (trips/vehicles) as possible.
%   (ii) Given the minimum #routes, minimize first-echelon travel distance.
%   (iii) A single route can visit multiple satellites (truck capacity permitting).
%   (iv) NO closing return depot is appended (output like [2,6], not [2,6,2]).
%
% INPUT
%   vrp2e : problem struct
%   seq_r : 2nd-echelon route encoding (NO return-to-satellite), where
%           node <= num_sat : satellite marker
%           node >  num_sat : customer node (global index = num_sat + cus_id)
%
% OUTPUT
%   seq_sat : first-echelon encoding (NO return-to-depot), concatenated as
%             [dep, sat+num_dep, sat+num_dep, dep, ...]
%   cap_sat : same length as seq_sat; delivered load at satellite nodes, 0 at dep nodes
%
% Notes
%   - Instances have single-digit depots/satellites (your statement), so enumeration is cheap.
%   - If a satellite total required load > truck capacity, we split it into multiple tasks.

% ---------------- basics ----------------
num_dep = length(vrp2e.rad_ds(:,1));
num_sat = vrp2e.num_sat;
num_cus = vrp2e.num_cus;

fleet   = vrp2e.fleet;
cap_trk = fleet(1,1);

dis_ds  = vrp2e.dis_ds;
demand  = vrp2e.demand;
type    = vrp2e.type;

seq_sat = [];
cap_sat = [];

% ---------------- parse 2nd-echelon to get sat->customers assignment ----------------
seq_r = seq_r(seq_r>0);
if isempty(seq_r)
    return;
end

sat_of_cus = zeros(1,num_cus);
cur_sat = 0;
for k = 1:numel(seq_r)
    n = seq_r(k);
    if n <= num_sat
        cur_sat = n;
    else
        c = n - num_sat;
        if c >= 1 && c <= num_cus
            sat_of_cus(c) = cur_sat;
        end
    end
end

used_sats = unique(sat_of_cus(sat_of_cus>0));
if isempty(used_sats)
    return;
end

% ---------------- compute required load per satellite ----------------
load_sat = zeros(1,num_sat);
for s = used_sats(:)'
    cus_s = find(sat_of_cus==s);
    if isempty(cus_s)
        continue;
    end
    % Keep consistent with your original indexing:
    % type is indexed by customer id (1..num_cus), demand by global node (num_sat+cus).
    del_mask = false(size(cus_s));
    try
        del_mask = strcmp(type(cus_s),'delivery');
    catch
        % if type is not available, assume all are deliveries
        del_mask(:) = true;
    end
    if any(del_mask)
        idx = cus_s(del_mask) + num_sat;
        idx = idx(idx>=1 & idx<=numel(demand));
        load_sat(s) = sum(demand(idx));
    end
end

% Keep only satellites with positive load
used_sats = used_sats(load_sat(used_sats) > 0);
if isempty(used_sats)
    return;
end

% ---------------- build tasks (sat, load) with load<=cap_trk ----------------
% Each task corresponds to "one visit to a satellite" in one route.
% If total load at a satellite exceeds capacity, it is split into multiple tasks.

task_sat  = [];
task_load = [];
for s = used_sats(:)'
    q = load_sat(s);
    while q > cap_trk + 1e-12
        task_sat(end+1)  = s; %#ok<AGROW>
        task_load(end+1) = cap_trk; %#ok<AGROW>
        q = q - cap_trk;
    end
    if q > 1e-12
        task_sat(end+1)  = s; %#ok<AGROW>
        task_load(end+1) = q; %#ok<AGROW>
    end
end

T = numel(task_sat);
if T == 0
    return;
end

% ---------------- exact enumeration over depot assignment (small T) ----------------
% Objective: lexicographic (min #routes, then min distance)

max_enum = 50000;  % safe guard
if (num_dep^T) > max_enum
    % fallback: nearest depot for each task
    assign_best = zeros(1,T);
    for t = 1:T
        s = task_sat(t);
        [~, d0] = min(dis_ds(1:num_dep, num_dep+s));
        assign_best(t) = d0;
    end
    [seq_sat,cap_sat] = build_solution_from_assignment(assign_best, task_sat, task_load, vrp2e);
    return;
end

bestTrips = inf;
bestDist  = inf;
bestAssign = [];

% enumerate in base-num_dep counting (1..num_dep)
assign = ones(1,T);
num_total = num_dep^T;
for it = 1:num_total
    % evaluate this assignment
    [ok, trips, dist] = eval_assignment(assign, task_sat, task_load, vrp2e);
    if ok
        if (trips < bestTrips) || (trips==bestTrips && dist < bestDist - 1e-12)
            bestTrips = trips;
            bestDist  = dist;
            bestAssign = assign;
        end
    end

    % increment base-num_dep number
    assign = next_assignment(assign, num_dep);
end

if isempty(bestAssign)
    % fallback to nearest depot assignment if something unexpected happens
    bestAssign = zeros(1,T);
    for t = 1:T
        s = task_sat(t);
        [~, d0] = min(dis_ds(1:num_dep, num_dep+s));
        bestAssign(t) = d0;
    end
end

[seq_sat,cap_sat] = build_solution_from_assignment(bestAssign, task_sat, task_load, vrp2e);

end

% ======================================================================
% Helpers
% ======================================================================

function assign2 = next_assignment(assign, base)
% increment vector in base 'base', digits in [1..base]
assign2 = assign;
for i = 1:numel(assign2)
    assign2(i) = assign2(i) + 1;
    if assign2(i) <= base
        return;
    end
    assign2(i) = 1;
end
end

function [ok, trips, dist] = eval_assignment(assign, task_sat, task_load, vrp2e)
num_dep = length(vrp2e.rad_ds(:,1));
trips = 0;
dist  = 0;
ok = true;

for d = 1:num_dep
    idx = find(assign==d);
    if isempty(idx), continue; end

    sats  = task_sat(idx);
    loads = task_load(idx);

    [ok_d, trips_d, dist_d] = solve_depot_lexi(d, sats, loads, vrp2e);
    if ~ok_d
        ok = false; return;
    end
    trips = trips + trips_d;
    dist  = dist  + dist_d;
end
end

function [seq_sat,cap_sat] = build_solution_from_assignment(assign, task_sat, task_load, vrp2e)
num_dep = length(vrp2e.rad_ds(:,1));
seq_sat = [];
cap_sat = [];

for d = 1:num_dep
    idx = find(assign==d);
    if isempty(idx), continue; end

    sats  = task_sat(idx);
    loads = task_load(idx);

    [~, routes, caps] = solve_depot_routes(d, sats, loads, vrp2e);

    % concatenate (NO return depot)
    for r = 1:numel(routes)
        sat_order = routes{r};
        cap_order = caps{r};
        if isempty(sat_order)
            continue;
        end
        seq_sat = [seq_sat, d, (sat_order + num_dep)]; %#ok<AGROW>
        cap_sat = [cap_sat, 0, cap_order]; %#ok<AGROW>
    end
end
end

function [ok, trips, dist] = solve_depot_lexi(dep, sats, loads, vrp2e)
% return only metrics
[ok, routes, caps, dist] = solve_depot_routes(dep, sats, loads, vrp2e);
if ~ok
    trips = inf;
else
    trips = numel(routes);
end
end

function [ok, routes, caps, dist_best] = solve_depot_routes(dep, sats, loads, vrp2e)
% Solve per depot with lexicographic objective:
%   minimize #routes first, then distance.

cap_trk = vrp2e.fleet(1,1);
dis_ds  = vrp2e.dis_ds;
num_dep = length(vrp2e.rad_ds(:,1));

m = numel(sats);
if m == 0
    ok = true; routes = {}; caps = {}; dist_best = 0; return;
end

% Precompute subset feasibility + best open-path order and distance
nMask = bitshift(1,m);
fullMask = nMask - 1;
feasible = false(1,nMask);
subDist  = inf(1,nMask);
subOrder = cell(1,nMask);
subCaps  = cell(1,nMask);

for mask = 1:(nMask-1)
    idx = find(bitget(mask, 1:m));
    totalLoad = sum(loads(idx));
    if totalLoad > cap_trk + 1e-12
        continue;
    end

    sat_list = sats(idx);
    % (In your setting, a satellite won't appear twice in a feasible subset.)
    sat_list = sat_list(:)';

    [ord, dval] = tsp_open_path(dep, sat_list, dis_ds, num_dep);
    feasible(mask) = true;
    subDist(mask)  = dval;
    subOrder{mask} = ord;

    % caps for this subset in the same order
    cap_map = zeros(1, numel(ord));
    for i = 1:numel(ord)
        s = ord(i);
        j = find(sat_list==s, 1, 'first');
        if isempty(j)
            cap_map(i) = 0;
        else
            % sat_list corresponds 1-1 to idx positions
            cap_map(i) = loads(idx(j));
        end
    end
    subCaps{mask} = cap_map;
end

% DP over masks: dpTrips, dpDist
INF = 1e18;
dpTrips = INF*ones(1,nMask);
dpDist  = INF*ones(1,nMask);
preMask = zeros(1,nMask);
preSub  = zeros(1,nMask);

% start
idx0 = 1; % mask=0 stored at 1
dpTrips(idx0) = 0;
dpDist(idx0)  = 0;

for mask = 0:(nMask-1)
    curIdx = mask + 1;
    if dpTrips(curIdx) >= INF/2
        continue;
    end
    rem = bitxor(mask, fullMask);
    rem = bitand(rem, nMask-1);
    if rem == 0
        continue;
    end

    % iterate feasible subsets within remaining
    sub = rem;
    while sub > 0
        if feasible(sub)
            newMask = bitor(mask, sub);
            newIdx  = newMask + 1;

            tNew = dpTrips(curIdx) + 1;
            dNew = dpDist(curIdx) + subDist(sub);

            if (tNew < dpTrips(newIdx)) || (tNew==dpTrips(newIdx) && dNew < dpDist(newIdx) - 1e-12)
                dpTrips(newIdx) = tNew;
                dpDist(newIdx)  = dNew;
                preMask(newIdx) = mask;
                preSub(newIdx)  = sub;
            end
        end
        sub = bitand(sub-1, rem);
    end
end

fullIdx  = fullMask + 1;
if dpTrips(fullIdx) >= INF/2
    ok = false;
    routes = {};
    caps   = {};
    dist_best = inf;
    return;
end

% reconstruct
ok = true;
dist_best = dpDist(fullIdx);

routes_rev = {};
caps_rev   = {};
mask = fullMask;
while mask ~= 0
    idxm = mask + 1;
    sub  = preSub(idxm);
    ord  = subOrder{sub};
    capv = subCaps{sub};
    routes_rev{end+1} = ord; %#ok<AGROW>
    caps_rev{end+1}   = capv; %#ok<AGROW>
    mask = preMask(idxm);
end

% reverse to get forward order
routes = routes_rev(end:-1:1);
caps   = caps_rev(end:-1:1);

end

function [best_order, best_cost] = tsp_open_path(dep, sat_list, dis_ds, num_dep)
% Exact open-path TSP starting at depot (no return), visiting all satellites.
% sat_list: satellite ids (1..num_sat)

k = numel(sat_list);
if k == 0
    best_order = [];
    best_cost  = 0;
    return;
elseif k == 1
    s1 = sat_list(1);
    best_order = s1;
    best_cost  = dis_ds(dep, num_dep + s1);
    return;
end

best_cost = inf;
best_order = sat_list;

P = perms(sat_list);
for i = 1:size(P,1)
    p = P(i,:);
    cost = dis_ds(dep, num_dep + p(1));
    for j = 2:k
        cost = cost + dis_ds(num_dep + p(j-1), num_dep + p(j));
        if cost >= best_cost
            break;
        end
    end
    if cost < best_cost
        best_cost = cost;
        best_order = p;
    end
end
end
