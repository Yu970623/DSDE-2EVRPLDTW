%% ----------------------------- Main Program (LNS, with Time Windows & Bi-objective Archive) -----------------------------
% This file is intentionally DIFFERENT from the ALNS main:
%   - ALNS: adaptive operator selection (scores/weights updated online)
%   - LNS : Large Neighborhood Search with a NON-adaptive destroy/repair scheme + SA acceptance
% The lower-level functions (initial / repair1 / repair2 / local2 / mfirst / msecond / time_violate / updateArchive) are shared.

clear; close all; clc;
rng('shuffle');

%% ---------------- Parameters ----------------
max_run     = 12;          % number of independent runs
max_gen     = 1000;       % iterations per run
nreg        = 50;         % intensification frequency (local search every nreg iters)
nadir_point = [30,4;50,6;70,8;90,10];
% Penalty for constraint violation (kept same interface as ALNS)
pen_0  = 5 + rand*9995;
theta  = 0.02;

% LNS neighborhood size (remove q customers each iteration)
q_min_ratio = 0.08;       % 8% customers
q_max_ratio = 0.25;       % 25% customers

% Simulated annealing temperature schedule (for accepting worse moves)
T0_factor = 0.10;         % T0 = T0_factor * (initial scalarized objective)
T_alpha   = 0.995;        % T <- alpha*T each iteration

seq_dataset = [60,60,60,60];  % #instances in each folder (kept)

%% ---------------- Start ----------------
for num_folder = 1
    seq_data = seq_dataset(num_folder);
    for seq = 8
        % ---- Load data (same as ALNS main) ----
        [vrp2e,dis_ds,dis_sc,rad_ds,rad_sc,neib_ss,neib_cc,neib_sc,num_dep,num_sat,num_cus] = extractdata(num_folder,seq); %#ok<ASGLU>

        case_name       = vrp2e.case_name;

        data.Windows    = vrp2e.Windows;
        data.start_time = vrp2e.tw_start;
        data.end_time   = vrp2e.tw_end;
        data.Services   = vrp2e.service;

        data.dis_first  = dis_ds(:,num_dep+1:end);   % depot->sat
        data.dis_second = dis_sc(:,num_sat+1:end);   % (sat+cus)->cus (your original format)
        data.v          = [30,vrp2e.V];              % [truck_speed, drone_speed]
        data.T1         = 60*data.dis_first/data.v(1);
        data.T2         = 60*data.dis_second/data.v(2);
        data.CW         = [30,50];

        % ---- Store results for each run (keep the same result() interface) ----
        opt_vrps   = zeros(1,max_run);
        opt_vlts   = zeros(1,max_run);
        opt_cuss   = cell(1,max_run);
        opt_sats   = cell(1,max_run);
        opt_caps   = cell(1,max_run);
        vrps_covs  = cell(1,max_run);
        vltc_covs  = cell(1,max_run);
        cpu_time   = 0;

        % ND archive per run
        nd_objs   = cell(1,max_run);
        nd_vlts   = cell(1,max_run);
        nd_cuss   = cell(1,max_run);
        nd_sats   = cell(1,max_run);
        nd_caps   = cell(1,max_run);
        
        for nrun = 1
            fprintf('LNS on %s (run %d)\n',case_name,nrun);

            penal = pen_0;
            tic;

            %% ---- Initial solution ----
            [seq_cus,seq_sat,cap_sat] = initial(vrp2e);

            [e_cost2,c_cost2,c_weight,c_energy] = msecond(vrp2e,seq_cus,seq_sat,data);
            vlt2_basic = c_weight + c_energy;

            [e_cost1,c_cost1,vlt1] = mfirst(vrp2e,seq_sat,cap_sat);

            fvrp1 = e_cost1 + e_cost2;      % equipment-type cost
            fvrp2 = c_cost1 + c_cost2;      % operating/energy-type cost

            vlt_tw = time_violate(seq_sat, seq_cus, data, vrp2e);
            fvlt2  = vlt2_basic + vlt_tw;
            fvlt   = vlt1 + fvlt2;

            fvrp_scalar = fvrp1 + fvrp2;
            

            % update penalty (same logic as ALNS)
            if fvlt > 0
                penal = max(min(penal/1.1,10000),5.0);
            else
                penal = max(min(penal*1.1,10000),5.0);
            end

            % ---- incumbent best (scalarized) ----
            opt_vrp_scalar = fvrp_scalar;
            opt_vlt_scalar = fvlt;
            opt_cus        = seq_cus;
            opt_sat        = seq_sat;
            opt_cap        = cap_sat;

            % ---- ND archive init ----
            ArchObj = []; ArchVlt = [];
            ArchCus = {}; ArchSat = {}; ArchCap = {};
            [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                [fvrp1,fvrp2],fvlt,seq_cus,seq_sat,cap_sat);

            %% ---- LNS loop ----
            coverge      = [];
            vlt_converge = [];

            % Neighborhood size bounds (integer)
            q_min = max(1, round(q_min_ratio * num_cus));
            q_max = max(q_min, round(q_max_ratio * num_cus));

            % SA temperature
            T = max(1e-6, T0_factor * (fvrp_scalar + penal*fvlt));
            
            HV_value = [];
            c = [];
            for gen = 1:max_gen
                % --------- 1) Destroy (large neighborhood) ----------
                q = randi([q_min, q_max]);               % remove q customers
                mode_destroy = 1 + mod(gen-1, 2);        % cycle: 1-random, 2-related
                [seq_cs, seq_c_removed] = lns_destroy(vrp2e, seq_cus, q, mode_destroy);

                % --------- 2) Repair (non-adaptive) ----------
                % fixed alternation of repair mode (greedy / regret)
                if mod(gen,2)==1
                    sm_repair = 1;   % greedy
                else
                    sm_repair = 4;   % regret (falls into repair2 "sm>=4" branch)
                end
                if mod(gen,10)==0
                    if isempty(ArchObj)
                        HV_value = [HV_value,0];
                    else
                        HV_value = [HV_value,HV_out(ArchObj,nadir_point(4,:))];
                    end
                    c = [c,fvlt];
                end
                seq_cs = repair2(vrp2e, seq_cs, seq_c_removed, [], sm_repair);

                % optional cleanup to avoid degenerate "sat,sat" segments
                seq_cs = cleanup_second_seq(vrp2e, seq_cs);

                % --------- 3) Rebuild 1st echelon and evaluate ----------
                [seq_ss,cap_ss,e1,c1,vlt1_new] = repair1(vrp2e, seq_cs);

                [e2,c2,cw,ce] = msecond(vrp2e, seq_cs, seq_ss, data);
                vlt2_basic_new = cw + ce;

                vlt_tw_new = time_violate(seq_ss, seq_cs, data, vrp2e);
                fvlt_new   = vlt1_new + vlt2_basic_new + vlt_tw_new;

                f1_new = e1 + e2;
                f2_new = c1 + c2;
                f_scalar_new = f1_new + f2_new;

                % --------- 4) ND archive update (always) ----------
                [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                    [f1_new,f2_new],fvlt_new,seq_cs,seq_ss,cap_ss);

                % --------- 5) Penalty update ----------
                if fvlt_new > 0
                    penal = max(min(penal/1.1,10000),5.0);
                else
                    penal = max(min(penal*1.1,10000),5.0);
                end

                % --------- 6) Acceptance (SA on penalized scalar) ----------
                cur_score  = fvrp_scalar + penal*fvlt;
                new_score  = f_scalar_new + penal*fvlt_new;
                accept = false;

                if new_score < cur_score
                    accept = true;
                else
                    dE = new_score - cur_score;
                    if rand < exp(-dE / max(1e-9,T))
                        accept = true;
                    end
                end

                if accept
                    seq_cus     = seq_cs;
                    seq_sat     = seq_ss;
                    cap_sat     = cap_ss;
                    fvrp1       = f1_new;
                    fvrp2       = f2_new;
                    fvrp_scalar = f_scalar_new;
                    fvlt        = fvlt_new;
                    fvlt1       = vlt1_new;
                    fvlt2       = vlt2_basic_new + vlt_tw_new;
                end

                % --------- 7) Intensification (periodic local search) ----------
                if mod(gen, nreg) == 0
                    seq_tmp = local2(vrp2e, seq_cus);
                    [seq_tmp1,cap_tmp,e1t,c1t,vlt1t] = repair1(vrp2e, seq_tmp);
                    [e2t,c2t,cwt,cet] = msecond(vrp2e, seq_tmp, seq_tmp1, data);
                    vlt2t = cwt + cet + time_violate(seq_tmp1, seq_tmp, data, vrp2e);
                    f1t = e1t + e2t; f2t = c1t + c2t;
                    fsc_t = f1t + f2t;
                    fvlt_t = vlt1t + vlt2t;

                    % accept only if improves current penalized score (greedy intensification)
                    if fsc_t + penal*fvlt_t < fvrp_scalar + penal*fvlt
                        seq_cus = seq_tmp;
                        seq_sat = seq_tmp1;
                        cap_sat = cap_tmp;
                        fvrp1 = f1t; fvrp2 = f2t; fvrp_scalar = fsc_t;
                        fvlt = fvlt_t; fvlt1 = vlt1t; fvlt2 = vlt2t - vlt1t; %#ok<NASGU>
                    end

                    [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                        ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                        [fvrp1,fvrp2],fvlt,seq_cus,seq_sat,cap_sat);
                end

                % --------- 8) Best-so-far update (scalarized) ----------
                if fvrp_scalar + penal*fvlt < opt_vrp_scalar + penal*opt_vlt_scalar
                    opt_vrp_scalar = fvrp_scalar;
                    opt_vlt_scalar = fvlt;
                    opt_cus        = seq_cus;
                    opt_sat        = seq_sat;
                    opt_cap        = cap_sat;
                end

                % --------- 9) Temperature cooling & logs ----------
                T = T_alpha * T;

                coverge      = [coverge,opt_vrp_scalar]; %#ok<AGROW>
                vlt_converge = [vlt_converge,opt_vlt_scalar]; %#ok<AGROW>

                fprintf('LNS Set%d_num%d_gen%4d: f1=%.4f f2=%.4f vlt=%.2f (T=%.3g)\n', ...
                        num_folder,seq,gen,fvrp1,fvrp2,fvlt,T);
            end
            fileName1 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\LNS23100HV.mat');
            fileName2 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\LNS23100CV.mat');
            save(fileName1, 'HV_value', '-V7.3');
            save(fileName2, 'c', '-V7.3');

            end_sat = compute_end_sat(vrp2e, opt_cus);

            cpu_time        = cpu_time + toc;
            opt_vrps(nrun)  = opt_vrp_scalar;
            opt_vlts(nrun)  = opt_vlt_scalar;
            opt_cuss{nrun}  = {opt_cus; end_sat};   % keep consistent with ALNS for plotting
            opt_sats{nrun}  = opt_sat;
            opt_caps{nrun}  = opt_cap;
            vrps_covs{nrun} = coverge;
            vltc_covs{nrun} = vlt_converge;

            nd_objs{nrun} = ArchObj;
            nd_vlts{nrun} = ArchVlt;
            nd_cuss{nrun} = ArchCus;
            nd_sats{nrun} = ArchSat;
            nd_caps{nrun} = ArchCap;
        end

        cpu_time = cpu_time / max_run;
        fprintf('Average Time Spend (LNS):  %6.2f\n\n',cpu_time);

        name = strcat('LNS_','Set',num2str(num_folder),'_','num',num2str(seq));
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps, ...
               vrps_covs,vltc_covs,cpu_time);

        save([name '_NDarchive.mat'],'nd_objs','nd_vlts','nd_cuss','nd_sats','nd_caps');
    end
end

%% ======================== Helper Functions ========================

function [seq_keep, removed] = lns_destroy(vrp2e, seq, q, mode)
% LNS destroy operator (NON-adaptive)
% mode=1: random removal
% mode=2: related removal (remove spatially-near customers w.r.t a seed)
    num_sat = vrp2e.num_sat;
    dis_sc  = vrp2e.dis_sc;

    idxCus = find(seq > num_sat);
    if isempty(idxCus)
        seq_keep = seq;
        removed  = [];
        return;
    end

    q = min(q, numel(idxCus));
    if q <= 0
        seq_keep = seq;
        removed  = [];
        return;
    end

    if mode == 1 || numel(idxCus) == 1
        pick = idxCus(randperm(numel(idxCus), q));
    else
        % related removal: pick a seed customer, then remove its nearest (q-1)
        seed_pos = idxCus(randi(numel(idxCus)));
        seed_node = seq(seed_pos);  % global node id (sat+cus)
        cand_nodes = seq(idxCus);

        % distance from seed to each candidate node (use dis_sc row)
        % dis_sc is (sat+cus) x (sat+cus) or (sat+cus) x ? in your dataset;
        % here we only need "node-node" distance, which is usually available.
        try
            d = dis_sc(seed_node, cand_nodes);
        catch
            % fallback: use column distance if row not available
            d = dis_sc(cand_nodes, seed_node)';
        end
        [~, order] = sort(d, 'ascend');
        pick = idxCus(order(1:q));
    end

    removed = seq(pick);
    seq_keep = seq;
    seq_keep(pick) = [];
end

function seq2 = cleanup_second_seq(vrp2e, seq)
% Remove empty routes / duplicated satellites, keep representation: [sat, cus..., sat, cus..., ...]
    num_sat = vrp2e.num_sat;
    if isempty(seq), seq2 = seq; return; end

    seq = seq(:)';  % row
    % remove leading customers (shouldn't happen, but safe)
    while ~isempty(seq) && seq(1) > num_sat
        seq(1) = [];
    end
    if isempty(seq), seq2 = seq; return; end

    % remove consecutive satellites (keep only the first)
    i = 2;
    while i <= numel(seq)
        if seq(i) <= num_sat && seq(i-1) <= num_sat
            seq(i) = [];
        else
            i = i + 1;
        end
    end

    % remove trailing satellite if it has no customers after it
    if numel(seq) >= 1 && seq(end) <= num_sat
        seq(end) = [];
    end
    seq2 = seq;
end

function end_sat = compute_end_sat(vrp2e, route)
% For each second-echelon subroute (starts at a satellite), return the nearest satellite to the last customer
    num_sat = vrp2e.num_sat;
    dis_sc  = vrp2e.dis_sc;

    end_sat = [];
    if isempty(route), return; end

    loc_sat = find(route <= num_sat);
    nRoute  = numel(loc_sat);
    end_sat = zeros(1,nRoute);

    for r = 1:nRoute
        sidx = loc_sat(r);
        if r < nRoute
            eidx = loc_sat(r+1) - 1;
        else
            eidx = numel(route);
        end
        last_node = route(eidx);

        if last_node > num_sat
            % nearest satellite to last_node
            try
                dists = dis_sc(last_node, 1:num_sat);
            catch
                dists = dis_sc(1:num_sat, last_node)';
            end
            [~,best_s] = min(dists);
            end_sat(r) = best_s;
        else
            end_sat(r) = route(sidx);
        end
    end
end
