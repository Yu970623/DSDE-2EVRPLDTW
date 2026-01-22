%% -----------------------------主程序（ALNS，带时间窗 & 双目标档案）-----------------------------
clear; close; clc;
rng('shuffle');

% ---- 参数设置和种群初始化 ----
max_run     = 12;            % 重复实验次数
max_gen     = 1000;          % 每次实验最多迭代次数
nreg        = 100;           % 每次迭代中局部优化次数
pen_0       = 5+rand*9995;   % 容量的惩罚值
delta       = 1.1;           % 惩罚值的调节系数
theta       = 0.02;          % 与最优解的比例系数
seq_dataset = [60,60,60,60]; % 每个数据集的样本数
nadir_point = [30,4;50,6;70,8;90,10];
% ---- Start ----
for num_folder = 1
    seq_data = seq_dataset(num_folder);
    for seq = 3
        % ---- 数据读取（带时间窗）----
        [vrp2e,coord_dep,coord_sat,coord_cus,fleet,timewindows,timeservice, ...
            demand,type,dis_ds,dis_sc,rad_ds,rad_sc, ...
            neib_ss,neib_cc,neib_sc,num_dep,num_sat,num_cus] = extractdata(num_folder,seq);

        case_name       = vrp2e.case_name;
        data.Windows    = vrp2e.Windows;
        data.start_time = vrp2e.tw_start;
        data.end_time   = vrp2e.tw_end;
        data.Services   = vrp2e.service;
        data.dis_first  = dis_ds(:,num_dep+1:end);   % 仓库→卫星
        data.dis_second = dis_sc(:,num_sat+1:end);   % 卫星→客户
        data.v          = [30,vrp2e.V];              % [卡车速度, 无人机速度]
        data.T1         = 60*data.dis_first/data.v(1);
        data.T2         = 60*data.dis_second/data.v(2);
        data.CW         = [30,50];                   % 时间窗超前/滞后权重

        % ---- 存储每次实验的结果（与原版保持接口一致）----
        opt_vrps   = zeros(1,max_run);     % 标量综合“最优值”（便于 result 使用）
        opt_vlts   = zeros(1,max_run);     % 最优违反量（标量）
        opt_cuss   = cell(1,max_run);
        opt_sats   = cell(1,max_run);
        opt_caps   = cell(1,max_run);
        vrps_covs  = cell(1,max_run);
        vltc_covs  = cell(1,max_run);
        cpu_time   = 0;

        % ---- 额外：每个 run 的非支配档案（双目标）----
        nd_objs   = cell(1,max_run);   % 每个元素是 [Nnd x 2] 的矩阵，列为 [f1,f2]
        nd_vlts   = cell(1,max_run);   % [Nnd x 1] 违反量
        nd_cuss   = cell(1,max_run);   % cell{nrun}{i} 为第 i 个非支配解的二级路径
        nd_sats   = cell(1,max_run);   % cell{nrun}{i} 为第 i 个非支配解的一级路径
        nd_caps   = cell(1,max_run);   % cell{nrun}{i} 为第 i 个非支配解的卫星容量

        % ---- 算法循环体 ----
        for nrun = 1
            fprintf('ALNS on %s (run %d)\n',case_name,nrun);
            score = ones(1,12);    % 3 全局破坏 + 5 局部破坏 + 4 修复，共 12 种模式
            penal = pen_0;

            tic;                   % 计时开始
            gen  = 0;
            %% ---- 初始解 ----
            [seq_cus,seq_sat,cap_sat]       = initial(vrp2e);
            [fe_cost2,fc_cost2,c_weight,c_energy] = msecond(vrp2e,seq_cus,seq_sat,data);
            fvlt2_basic = c_weight + c_energy;
            [fe_cost1,fc_cost1,fvlt1]       = mfirst(vrp2e,seq_sat,cap_sat);

            % 双目标（不再一开始就累加）
            fvrp1 = fe_cost1 + fe_cost2;   % 设备购置相关成本
            fvrp2 = fc_cost1 + fc_cost2;   % 运营成本

            % 时间窗违反量（加入第二层的违反中）
            vlt_tw = time_violate(seq_sat, seq_cus, data, vrp2e);
            fvlt2  = fvlt2_basic + vlt_tw;
            fvlt   = fvlt1 + fvlt2;

            % 标量化用于“搜索接受准则”（仍保持原逻辑）
            fvrp_scalar = fvrp1 + fvrp2;

            % ---- 更新惩罚值 ----
            if fvlt > 0
                penal = max(min(penal/1.1,10000),5.0);
            else
                penal = max(min(penal*1.1,10000),5.0);
            end

            sat_offl = ones(1,num_sat);   % 当前路径对应的卫星开闭状态
            s_offl   = ones(1,num_sat);   % 当前关闭卫星的状态
            s_offs   = [];                % 删除客户对应的卫星

            % ---- 当前 run 中的“标量最优解”（保留原标量接口）----
            opt_vrp_scalar = fvrp_scalar;
            opt_vlt_scalar = fvlt;
            opt_cus        = seq_cus;
            opt_sat        = seq_sat;
            opt_cap        = cap_sat;

            % ---- 非支配档案初始化（双目标 + 约束）----
            ArchObj = [];
            ArchVlt = [];
            ArchCus = {};
            ArchSat = {};
            ArchCap = {};
            [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                [fvrp1,fvrp2],fvlt,seq_cus,seq_sat,cap_sat);

            %% ---- 进化开始 ----
            ng            = 0;          % 局部搜索计数
            coverge       = [];
            vlt_converge  = [];
            HV_value = [];
            c = [];
            while gen < max_gen
                gen = gen + 1;

                %% ---- 第二层路径优化 ----
                % ---- 选取破坏模式 ----
                if ng == nreg
                    sm1 = roulette(score(1:3),1);           % 全局破坏 1–3
                    [seq_cs,seq_c,sat_offl,s_offs] = destroyl(vrp2e,seq_cus,sat_offl,sm1);
                else
                    sm1  = roulette(score(4:8),1) + 3;      % 局部破坏 4–8
                    s_offl = sat_offl;
                    [seq_cs,seq_c,s_offs,s_offl] = destroys(vrp2e,seq_cus,s_offl,sm1);
                end
                if mod(gen,10)==0
                    if isempty(ArchObj)
                        HV_value = [HV_value,0];
                    else
                        HV_value = [HV_value,HV_out(ArchObj,nadir_point(3,:))];
                    end
                    c = [c,fvlt];
                end
                % ---- 选取修复模式 ----
                sm2 = roulette(score(9:12),1);              % 修复 9–12
                off = 1:num_sat;
                if sm2 == 3
                    s_off = [repmat(off(s_offl==0),[numel(seq_c),1]),s_offs'];
                elseif sm1 == 7 && numel(s_offs)>0
                    s_off = [off(s_offl==0),s_offs(1)];
                else
                    s_off = off(s_offl==0);
                end

                % ---- 修复（第二层）----
                seq_cs = repair2(vrp2e,seq_cs,seq_c,s_off,sm2);
                sm2    = sm2 + 8;       % 修复方式的得分序号

                
                %% ---- 第一层路径优化 ----
                [seq_ss,cap_ss,e_cost1,c_cost1,vlt1_new] = repair1(vrp2e,seq_cs);

                % 用一级到达时刻精确重算二级能耗（含等待）
                [e_cost2,c_cost2,c_weight,c_energy] = msecond(vrp2e,seq_cs,seq_ss,data);
                vlt2_basic = c_weight + c_energy;

                % ---- 时间窗惩罚 ----
                vlt_tw = time_violate(seq_ss, seq_cs, data, vrp2e);
                vlt2   = vlt2_basic + vlt_tw;
                fvlt_new = vlt1_new + vlt2;

                % ---- 新解双目标 ----
                fvrp1_new = e_cost1 + e_cost2;
                fvrp2_new = c_cost1 + c_cost2;
                fvrp_scalar_new = fvrp1_new + fvrp2_new;

                % ---- 在修复（repair1 + 时间窗惩罚）之后，将候选解加入非支配档案 ----
                [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                    [fvrp1_new,fvrp2_new],fvlt_new,seq_cs,seq_ss,cap_ss);

                % ---- 更新惩罚值 ----
                if fvlt_new > 0
                    penal = max(min(penal/1.1,10000),5.0);
                else
                    penal = max(min(penal*1.1,10000),5.0);
                end

                %% ---- 优化选择（仍使用标量惩罚用于接受判据）----
                if ng == nreg
                    % --- local2 intensification with guard (do NOT accept if it worsens penalized score) ---
                    seq_cs0 = seq_cs;
                    [seq_ss0,cap_ss0,e_cost10,c_cost10,vlt10] = repair1(vrp2e,seq_cs0);
                    [e_cost20,c_cost20,c_weight0,c_energy0] = msecond(vrp2e,seq_cs0,seq_ss0,data);
                    vlt2_basic0 = c_weight0 + c_energy0;
                    vlt_tw0 = time_violate(seq_ss0, seq_cs0, data, vrp2e);
                    vlt20 = vlt2_basic0 + vlt_tw0;
                    fvlt0 = vlt10 + vlt20;
                    fvrp10 = e_cost10 + e_cost20;
                    fvrp20 = c_cost10 + c_cost20;
                    fscalar0 = fvrp10 + fvrp20;
                    
                    seq_cs_try = local2(vrp2e,seq_cs0);
                    [seq_ss_try,cap_ss_try,e_cost1_try,c_cost1_try,vlt1_try] = repair1(vrp2e,seq_cs_try);
                    [e_cost2_try,c_cost2_try,c_weight_try,c_energy_try] = msecond(vrp2e,seq_cs_try,seq_ss_try,data);
                    vlt2_basic_try = c_weight_try + c_energy_try;
                    vlt_tw_try = time_violate(seq_ss_try, seq_cs_try, data, vrp2e);
                    vlt2_try = vlt2_basic_try + vlt_tw_try;
                    fvlt_try = vlt1_try + vlt2_try;
                    fvrp1_try = e_cost1_try + e_cost2_try;
                    fvrp2_try = c_cost1_try + c_cost2_try;
                    fscalar_try = fvrp1_try + fvrp2_try;
                    
                    if fscalar_try + penal*fvlt_try < fscalar0 + penal*fvlt0
                        % accept local2-improved solution as current
                        seq_cs = seq_cs_try;
                        seq_ss = seq_ss_try;
                        cap_ss = cap_ss_try;
                        e_cost1 = e_cost1_try; c_cost1 = c_cost1_try;
                        e_cost2 = e_cost2_try; c_cost2 = c_cost2_try;
                        vlt1_new = vlt1_try;
                        vlt2_basic = vlt2_basic_try;
                        vlt_tw = vlt_tw_try;
                        vlt2 = vlt2_try;
                        fvlt_new = fvlt_try;
                        fvrp1_new = fvrp1_try;
                        fvrp2_new = fvrp2_try;
                        fvrp_scalar_new = fscalar_try;
                    
                        [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                            ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                            [fvrp1_new,fvrp2_new],fvlt_new,seq_cs,seq_ss,cap_ss);
                        seq_cus   = seq_cs;
                        seq_sat   = seq_ss;
                        cap_sat   = cap_ss;
                    end
                    sat_offl = s_offl;
                    ng       = 0;
                elseif fvrp_scalar_new + penal*fvlt_new < (1+theta)*(opt_vrp_scalar+penal*opt_vlt_scalar)
                    seq_cs = local2(vrp2e,seq_cs);
                    [e_cost2,c_cost2,c_weight,c_energy] = msecond(vrp2e,seq_cs,seq_ss,data);
                    vlt2_basic = c_weight + c_energy;
                    vlt_tw = time_violate(seq_ss, seq_cs, data, vrp2e);
                    vlt2 = vlt2_basic + vlt_tw;
                    fvlt_new = vlt1_new + vlt2;
                    fvrp1_new       = e_cost1 + e_cost2;
                    fvrp2_new       = c_cost1 + c_cost2;
                    fvrp_scalar_new = fvrp1_new + fvrp2_new;

                    % ---- 邻域搜索（local2）之后：针对改进候选解更新一次档案 ----
                    fvlt_new_loc = vlt1_new + vlt2;   % 仅用于档案的可行性判断
                    [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                        ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                        [fvrp1_new,fvrp2_new],fvlt_new_loc,seq_cs,seq_ss,cap_ss);
                end

                % ---- 当前解更新 ----
                if fvrp_scalar_new + penal*fvlt_new < fvrp_scalar + penal*fvlt
                    seq_cus      = seq_cs;
                    seq_sat      = seq_ss;
                    cap_sat      = cap_ss;
                    fvrp1        = fvrp1_new;
                    fvrp2        = fvrp2_new;
                    fvrp_scalar  = fvrp_scalar_new;
                    fvlt1        = vlt1_new;
                    fvlt2        = vlt2;
                    fvlt         = fvlt_new;
                    sat_offl     = s_offl;
                    ng           = 0;
                else
                    ng           = ng + 1;
                end

                % ---- “标量意义下”的最优解（兼容原接口）----
                if fvrp_scalar + penal*fvlt < opt_vrp_scalar + penal*opt_vlt_scalar
                    opt_vrp_scalar = fvrp_scalar;
                    opt_vlt_scalar = fvlt;
                    opt_cus        = seq_cus;
                    opt_sat        = seq_sat;
                    opt_cap        = cap_sat;
                    score(sm1)     = score(sm1) + 1;
                    score(sm2)     = score(sm2) + 1;
                end

                % ---- 非支配档案更新（双目标 + 约束）当前解 ----
                [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
                    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                    [fvrp1,fvrp2],fvlt,seq_cus,seq_sat,cap_sat);

                coverge      = [coverge,opt_vrp_scalar];
                vlt_converge = [vlt_converge,opt_vlt_scalar];
                fprintf('ALNS Set%d_num%d_gen%4d: f1=%.4f f2=%.4f vlt=%.2f\n', ...
                        num_folder,seq,gen,fvrp1,fvrp2,fvlt);
            end
            fileName1 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\ALNS2350HV.mat');
            fileName2 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\ALNS2350CV.mat');
            save(fileName1, 'HV_value', '-V7.3');
            save(fileName2, 'c', '-V7.3');

            end_sat = compute_end_sat(vrp2e, opt_cus);
            cpu_time           = cpu_time + toc;
            opt_vrps(nrun)     = opt_vrp_scalar;
            opt_vlts(nrun)     = opt_vlt_scalar;
            opt_cuss{nrun}     = {opt_cus;end_sat};
            opt_sats{nrun}     = opt_sat;
            opt_caps{nrun}     = opt_cap;
            vrps_covs{nrun}    = coverge;
            vltc_covs{nrun}    = vlt_converge;

            % 保存本 run 的非支配解集（双目标）
            nd_objs{nrun} = ArchObj;
            nd_vlts{nrun} = ArchVlt;
            nd_cuss{nrun} = ArchCus;
            nd_sats{nrun} = ArchSat;
            nd_caps{nrun} = ArchCap;
        end

        cpu_time = cpu_time/max_run;
        fprintf('Average Time Spend (ALNS):  %6.2f\n\n',cpu_time);

        name = strcat('ALNS_','Set',num2str(num_folder),'_','num',num2str(seq));
        % 这里仍然给 result 传“标量最优值”，不破坏原接口
        result(vrp2e,name,opt_vrps,opt_vlts,opt_cuss,opt_sats,opt_caps, ...
               vrps_covs,vltc_covs,cpu_time);

        % 如果你后面需要用 nd_objs 等画 Pareto，可以在这里 save 到 mat 文件
        save([name '_NDarchive.mat'],'nd_objs','nd_vlts','nd_cuss','nd_sats','nd_caps');
    end
end


function end_sat = compute_end_sat(vrp2e, route)
    num_sat = vrp2e.num_sat;
    dis_sc  = vrp2e.dis_sc;

    end_sat = [];
    if isempty(route), return; end

    loc_sat = find(route <= num_sat);
    nRoute  = numel(loc_sat);
    end_sat = zeros(1,nRoute);

    for r = 1:nRoute
        s = loc_sat(r);
        if r < nRoute
            e = loc_sat(r+1)-1;
        else
            e = numel(route);
        end

        last_node = route(e);
        if last_node > num_sat
            dists = dis_sc(last_node,1:num_sat);
            [~,best_s] = min(dists);
            end_sat(r) = best_s;
        else
            end_sat(r) = route(s); % 极端：没客户
        end
    end
end