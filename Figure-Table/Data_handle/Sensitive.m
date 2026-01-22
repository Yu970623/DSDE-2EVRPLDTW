

% ===================== Helper: pick best solution across 8 runs =====================
% 规则：对每个run，先取该run内 sum(f1,f2) 最小的解；
% 然后在8个run之间再取 sum(f1,f2) 最小的那个run对应的解。

num_problems   = 4;
num_runs       = 8;

algorithm_name = {'DSDE_1W0.8E','DSDE_1W0.9E','DSDE_1W1E','DSDE_1W1.1E','DSDE_1W1.2E'};
lables         = {'Ca1_23_15','Ca1_23_30','Ca1_23_50','Ca1_35_100'};

% 每个问题的 depot 数（用于 Activated_Satellites 统计阈值）
num_dep = [2,2,2,3];

% 你的输出列顺序：算法 1、2、4、5（每个算法两列：value, change rate）
alg_list = [1,2,4,5];



outMat = nan(num_problems*4, 8);


function best = getBestAcrossRuns(algoFolder, labelStr, dep_th)
    rootDir  = 'F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data2';
    best.score = inf;
    best.f1 = nan; best.f2 = nan;
    best.droneRoutes = nan; best.actSats = nan;

    for rr = 1:8
        fpath = fullfile(rootDir, algoFolder, ['DSDE_', labelStr, '_TW_M2_D10_', num2str(rr), '.mat']);
        S = load(fpath);

        % 兼容你的数据结构命名：result 在 mat 里
        result = S.result;

        % run内：20个解的 f1+f2
        objective_sums = sum(result{1,2}.objs, 2);
        [minSumInRun, idx] = min(objective_sums);

        % 取这个解对应的各项（四个值来自同一解）
        objs = result{1,2}(1, idx).obj;
        f1   = objs(1);
        f2   = objs(2);

        droneRoutes = numel(result{1,2}(1, idx).add{1,1}{3,1});
        actSats     = sum(result{1,2}(1, idx).add{1,1}{1,1} > dep_th);

        % run间：比较该run最优解的 f1+f2
        if minSumInRun < best.score
            best.score = minSumInRun;
            best.f1 = f1; best.f2 = f2;
            best.droneRoutes = droneRoutes;
            best.actSats = actSats;
        end
    end
end

% ===================== 1) Compute baselines (Algorithm 3) per problem =====================
baseline = repmat(struct('f1',nan,'f2',nan,'droneRoutes',nan,'actSats',nan), 1, num_problems);

for j = 1:num_problems
    b = getBestAcrossRuns(algorithm_name{3}, lables{j}, num_dep(j)); % algo 3 baseline
    baseline(j).f1 = b.f1;
    baseline(j).f2 = b.f2;
    baseline(j).droneRoutes = b.droneRoutes;
    baseline(j).actSats = b.actSats;
end

% ===================== 2) Fill 16x8 output =====================
for j = 1:num_problems
    rows = (j-1)*4 + (1:4);  % 4行：f1,f2,droneRoutes,actSats

    for aa = 1:numel(alg_list)
        algIdx = alg_list(aa);
        colV = (aa-1)*2 + 1;   % value列
        colC = (aa-1)*2 + 2;   % change rate列

        r = getBestAcrossRuns(algorithm_name{algIdx}, lables{j}, num_dep(j));

        % value
        vals = [r.f1; r.f2; r.droneRoutes; r.actSats];

        % change rate vs baseline(algorithm 3)
        baseVals = [baseline(j).f1; baseline(j).f2; baseline(j).droneRoutes; baseline(j).actSats];

        chg = (vals - baseVals) ./ baseVals;

        % 写入 outMat
        outMat(rows, colV) = vals;
        outMat(rows, colC) = chg;
    end
end

% ===================== 3) Export to Excel =====================
outFile = fullfile(rootDir, 'E_16x8.xlsx'); % 你也可以改路径
writematrix(outMat, outFile);

disp(['Exported: ', outFile]);
