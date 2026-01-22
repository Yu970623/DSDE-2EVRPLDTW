clc; clear; close all;

%% ===================== 用户配置 =====================
algo_name  = 'DSDE';

mat_base   = 'F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data\';
xlsx_path  = 'F:\Onedrive\Gurobi\15_Gurobi.xlsx';

problems = { ...
    'Ca1_23_15_TW', 'Ca1_35_15_TW', 'Ca1_64_15_TW', ...
    'Ca2_23_15_TW', 'Ca2_35_15_TW', 'Ca2_64_15_TW', ...
    'Ca3_23_15_TW', 'Ca3_35_15_TW', 'Ca3_64_15_TW', ...
    'Ca4_23_15_TW', 'Ca4_35_15_TW', 'Ca4_64_15_TW', ...
    'Ca5_23_15_TW', 'Ca5_35_15_TW', 'Ca5_64_15_TW' ...
    'Ca1_23_30_TW', 'Ca1_35_30_TW', 'Ca1_64_30_TW', ...
    'Ca2_23_30_TW', 'Ca2_35_30_TW', 'Ca2_64_30_TW', ...
    'Ca3_23_30_TW', 'Ca3_35_30_TW', 'Ca3_64_30_TW', ...
    'Ca4_23_30_TW', 'Ca4_35_30_TW', 'Ca4_64_30_TW', ...
    'Ca5_23_30_TW', 'Ca5_35_30_TW', 'Ca5_64_30_TW' ...
};

nRuns   = 8;        % 每个 instance 8 个 .mat（_1.._8）
ArcN    = 100;      % ArchiveUpdate(Population, N)
ndigits = 2;        % Summary sheet 里 f1/f2 显示小数位
roundDigitsForUnique = 2; % DSDE 去重用的 round 位数（你想更强去重就加大/减小）

%% ===================== 读 Gurobi Excel =====================
T = readtable(xlsx_path, 'VariableNamingRule','preserve');

%% ===================== 主循环：逐 instance 统计 =====================
nP = numel(problems);

ProblemCol        = strings(nP,1);

ND_DSDE_Count     = zeros(nP,1);
ND_Gurobi_Count   = zeros(nP,1);

% 你要的：DSDE 与 Gurobi ND 解 “非支配（非劣）比率”
% R = |{ q in Q : exists p in P, q does NOT dominate p }| / |Q|
NoninferRate_DSDE_vs_Gurobi = nan(nP,1);

% 你要保留：只要存在一个 DSDE 点不被整个 Gurobi 集支配，就记 1
Hit_noninfer_DSDE_vs_Gurobi = zeros(nP,1);

ND_DSDE_Text      = strings(nP,1);
ND_Gurobi_Text    = strings(nP,1);

% 详细 ND 解（写 sheet）
DSDE_rows   = {};
Gurobi_rows = {};
DSDE_rows(1,:)   = {'Problem','k','f1','f2','First-echelon','Second-echelon'};
Gurobi_rows(1,:) = {'Problem','k','f1','f2','First-echelon','Second-echelon'};

for p = 1:nP
    probTag = problems{p};
    ProblemCol(p) = string(probTag);

    %% ---------- 1) DSDE：8 次运行合并 -> ArchiveUpdate(.,ArcN) ----------
    Population = [];
    for j = 1:nRuns
        mat_file = fullfile(mat_base, algo_name, ...
            sprintf('%s_%s_M2_D10_%d.mat', algo_name, probTag, j));

        if ~isfile(mat_file)
            warning('[%s] missing mat: %s', probTag, mat_file);
            continue
        end

        S = load(mat_file);
        if ~isfield(S,'result') || numel(S.result) < 2
            warning('[%s] invalid result in %s', probTag, mat_file);
            continue
        end

        Pop = S.result{1,2};
        Population = [Population, Pop];
    end

    % 直接得到 DSDE ND 解集（按你要求：非支配排序直接调用 ArchiveUpdate 即可）
    Population = ArchiveUpdate(Population, ArcN);

    % DSDE ND 点 + 路径
    if isempty(Population)
        DSDE_objs = [];
        DSDE_unique_objs = [];
    else
        DSDE_objs = get_pop_objs(Population);  % Nx2

        % ====== DSDE 去重（round + unique）======
        DSDE_round = round(DSDE_objs, roundDigitsForUnique);
        [DSDE_unique_round, iaU] = unique(DSDE_round, 'rows', 'stable');
        DSDE_unique_objs = DSDE_objs(iaU, :);           % 保留原始精度的 objs（但按 round 后去重）
        Population_unique = Population(iaU);            % 同步保留对应的路径

        % 写 DSDE 路径表
        for k = 1:length(Population_unique)
            sol = Population_unique(k);
            f1 = DSDE_unique_objs(k,1);
            f2 = DSDE_unique_objs(k,2);

            % 你指定的接口
            first_route  = sol.add{1,1}{1,1};
            second_route = sol.add{1,1}{2,1};

            DSDE_rows(end+1,:) = {probTag, k, f1, f2, ...
                format_route(first_route), format_route(second_route)}; %#ok<AGROW>
        end
    end

    %% ---------- 2) Gurobi：该 instance 的所有权重结果 -> 去重 -> 取 ND ----------
    [G_all, G_first, G_second] = get_gurobi_points_and_routes(T, probTag);

    if isempty(G_all)
        G_ND = [];
        G_first_ND = {};
        G_second_ND = {};
    else
        % 去重：按 (obj1,obj2) 去重，同时保留对应路径（取首次出现）
        [G_uniq, ia] = unique(G_all, 'rows', 'stable');
        first_uniq   = G_first(ia);
        second_uniq  = G_second(ia);

        % 权重 sweep 后取 ND 点集（你要求加进去）
        FrontNo = NDSort(G_uniq, 1);
        idxND   = find(FrontNo==1);

        G_ND = G_uniq(idxND,:);
        G_first_ND  = first_uniq(idxND);
        G_second_ND = second_uniq(idxND);

        for k = 1:size(G_ND,1)
            Gurobi_rows(end+1,:) = {probTag, k, G_ND(k,1), G_ND(k,2), ...
                string(G_first_ND{k}), string(G_second_ND{k})}; %#ok<AGROW>
        end
    end

    %% ---------- 3) 你要的“非支配（非劣）比率” + Hit_noninfer ----------
    % 说明：
    % - 对每个 Gurobi ND 点 q，若存在 DSDE 点 p 使得 q 不支配 p（q ⊀ p），则该 q 计入分子
    % - NoninferRate = 分子 / |Q|
    % - Hit_noninfer：是否存在一个 DSDE 点 p 不被任何 Gurobi 点支配（forall q, q ⊀ p）

    % P = DSDE_unique_objs（去重后）
    P = DSDE_unique_objs;
    Q = G_ND;

    [NoninferRate_DSDE_vs_Gurobi(p), Hit_noninfer_DSDE_vs_Gurobi(p)] = ...
        noninfer_rate_and_hit(P, Q);

    %% ---------- 4) 汇总 ----------
    ND_DSDE_Count(p)   = size(P,1);
    ND_Gurobi_Count(p) = size(Q,1);

    ND_DSDE_Text(p)   = format_points(P, ndigits);
    ND_Gurobi_Text(p) = format_points(Q, ndigits);

    fprintf('[%s] ND_DSDE=%d, ND_G=%d, NoninferRate=%.1f%%, Hit=%d\n', ...
        probTag, ND_DSDE_Count(p), ND_Gurobi_Count(p), ...
        100*NoninferRate_DSDE_vs_Gurobi(p), Hit_noninfer_DSDE_vs_Gurobi(p));
end

%% ===================== 写 Excel =====================
Out = table(ProblemCol, ...
    ND_DSDE_Count, NoninferRate_DSDE_vs_Gurobi, Hit_noninfer_DSDE_vs_Gurobi, ND_DSDE_Text, ...
    ND_Gurobi_Count, ND_Gurobi_Text, ...
    'VariableNames', { ...
      'Problem', ...
      'ND_count_DSDE', 'ND_rate_DSDE_noninfer_vs_GurobiND', 'Hit_noninfer_DSDE_vs_Gurobi', 'ND_solutions_DSDE', ...
      'ND_count_Gurobi', 'ND_solutions_Gurobi' });

out_xlsx = fullfile(pwd, sprintf('Noninfer_%s_vs_Gurobi_withRoutes.xlsx', algo_name));
writetable(Out, out_xlsx, 'Sheet', 'Summary');
writecell(DSDE_rows, out_xlsx, 'Sheet', 'DSDE_ND_routes');
writecell(Gurobi_rows, out_xlsx, 'Sheet', 'Gurobi_ND_routes');

fprintf('[save] Excel saved: %s\n', out_xlsx);

%% ===================== 函数区 =====================

function objs = get_pop_objs(Pop)
    % 兼容 PlatEMO 不同版本：Pop.objs 可能是属性或方法
    try
        objs = Pop.objs;      % 属性（最常见）
    catch
        objs = Pop.objs();    % 方法
    end
end

function [G, FirstE, SecondE] = get_gurobi_points_and_routes(T, probTag)
    instNames = string(T.("Instances"));
    keyProb   = norm_key(probTag);

    mask = false(size(instNames));
    for i = 1:numel(instNames)
        k = norm_key(instNames(i));
        if contains(k, keyProb) || contains(keyProb, k)
            mask(i) = true;
        end
    end

    if ~any(mask)
        G = [];
        FirstE = {};
        SecondE = {};
        return
    end

    obj1 = str2double(string(T.("obj1")(mask)));
    obj2 = str2double(string(T.("obj2")(mask)));
    valid = ~isnan(obj1) & ~isnan(obj2);

    G = [obj1(valid), obj2(valid)];

    % 注意：列名要与你的 Excel 一致
    FirstE_raw  = string(T.("First-echelon")(mask));
    SecondE_raw = string(T.("Second-echelon")(mask));

    FirstE  = cellstr(FirstE_raw(valid));
    SecondE = cellstr(SecondE_raw(valid));
end

function k = norm_key(s)
    s = lower(string(s));
    s = regexprep(s, '\.csv$', '');
    k = regexprep(s, '[^a-z0-9]', '');
end

function [rate, hit] = noninfer_rate_and_hit(P, Q)
    % rate = |{ p in P : exists q in Q, q does NOT dominate p }| / |P|
    % hit  = 1 if exists p in P and exists q in Q such that q does NOT dominate p
    %
    % minimization, dominance: q dominates p iff all(q<=p) and any(q<p)

    if isempty(P)
        rate = NaN;  % 或 0，按你习惯
        hit  = 0;
        return
    end
    if isempty(Q)
        rate = 1.0;
        hit  = 1;
        return
    end

    good = false(size(P,1),1);

    for i = 1:size(P,1)
        p = P(i,:);

        % 对每个 q 判断：q 是否支配 p
        q_dom_p = all(Q <= p, 2) & any(Q < p, 2);

        % 只要存在一个 q 不支配 p，则 good=1
        good(i) = any(~q_dom_p);
    end

    rate = mean(good);
    hit  = double(any(good));
end



function s = format_points(P, ndigits)
    if isempty(P)
        s = "";
        return
    end
    fmt = sprintf('%%.%df', ndigits);
    parts = strings(size(P,1),1);
    for i = 1:size(P,1)
        parts(i) = "[" + sprintf(fmt,P(i,1)) + "," + sprintf(fmt,P(i,2)) + "]";
    end
    s = strjoin(parts, "; ");
end

function str = format_route(r)
    % 兼容 numeric / cell / char / string
    if isempty(r)
        str = "";
        return
    end

    if isstring(r) || ischar(r)
        str = string(r);
        return
    end

    if isnumeric(r)
        str = strjoin(string(r(:).'), " ");
        return
    end

    if iscell(r)
        % 可能是“多条子路段”的 cell，每个 cell 里是数列
        segs = strings(numel(r),1);
        for i = 1:numel(r)
            ri = r{i};
            if isnumeric(ri)
                segs(i) = strjoin(string(ri(:).'), " ");
            elseif isstring(ri) || ischar(ri)
                segs(i) = string(ri);
            else
                segs(i) = "<unparsed>";
            end
        end
        str = strjoin(segs, " | ");
        return
    end

    str = "<unparsed>";
end

function Population = ArchiveUpdate(Population,N)
    %% Select feasible solutions
    fIndex     = all(Population.cons <= 0,2);
    Population = Population(fIndex);
    if isempty(Population)
        return
    else
        Population = Population(NDSort(Population.objs,1)==1);
        Population = Population(randperm(length(Population)));
        PCObj = Population.objs;
        nND   = length(Population);

        %% Population maintenance
        if length(Population) > N
            % Normalization
            fmax  = max(PCObj,[],1);
            fmin  = min(PCObj,[],1);
            PCObj = (PCObj-repmat(fmin,nND,1))./repmat(fmax-fmin,nND,1);

            % Determine the radius of the niche
            d  = pdist2(PCObj,PCObj);
            d(logical(eye(length(d)))) = inf;
            sd = sort(d,2);
            r  = mean(sd(:,min(3,size(sd,2))));
            R  = min(d./r,1);

            % Delete solution one by one
            while length(Population) > N
                [~,worst]  = max(1-prod(R,2));
                Population(worst)  = [];
                R(worst,:) = [];
                R(:,worst) = [];
            end
        end
    end
end
