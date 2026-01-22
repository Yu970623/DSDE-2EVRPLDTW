% ===================== Settings =====================
% 横轴：无人机续航 E，纵轴：无人机载重 W
algorithm_name = {'DSDE_0.8W0.8E','DSDE_0.8W0.9E','DSDE_0.8W1E','DSDE_0.8W1.1E','DSDE_0.8W1.2E', ...
                  'DSDE_0.9W0.8E','DSDE_0.9W0.9E','DSDE_0.9W1E','DSDE_0.9W1.1E','DSDE_0.9W1.2E', ...
                  'DSDE_1W0.8E','DSDE_1W0.9E','DSDE_1W1E','DSDE_1W1.1E','DSDE_1W1.2E', ...
                  'DSDE_1.1W0.8E','DSDE_1.1W0.9E','DSDE_1.1W1E','DSDE_1.1W1.1E','DSDE_1.1W1.2E', ...
                  'DSDE_1.2W0.8E','DSDE_1.2W0.9E','DSDE_1.2W1E','DSDE_1.2W1.1E','DSDE_1.2W1.2E'};

lables = {'Ca1_23_15','Ca1_23_30','Ca1_23_50','Ca1_35_100'};
num_dep = [2,2,2,3];

num_problems = numel(lables);

% 你热力图轴上想显示的百分比（仅用于显示，不影响读取）
battery_capacity_variations = [-20, -10, 0, 10, 20]; % E 轴
payload_capacity_variations = [-20, -10, 0, 10, 20]; % W 轴

% 你文件夹名里用的是倍率：0.8,0.9,1,1.1,1.2
E_levels = [0.8, 0.9, 1.0, 1.1, 1.2];
W_levels = [0.8, 0.9, 1.0, 1.1, 1.2];

% ===================== Helper: parse "DSDE_1.1W0.9E" -> W=1.1, E=0.9 =====================
% 注意：你给的命名规则是 "..._xWyE"
parseWE = @(s) deal( ...
    str2double(extractBetween(s, "DSDE_", "W")), ...
    str2double(extractBetween(s, "W", "E")) );

% ===================== getBestAcrossRuns: keep your function as is =====================
function best = getBestAcrossRuns(algoFolder, labelStr, dep_th)
    rootDir  = 'F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data2';
    best.score = inf;
    best.f1 = nan; best.f2 = nan;
    best.droneRoutes = nan; best.actSats = nan;

    for rr = 1:8
        fpath = fullfile(rootDir, algoFolder, ['DSDE_', labelStr, '_TW_M2_D10_', num2str(rr), '.mat']);
        S = load(fpath);
        result = S.result;

        objective_sums = sum(result{1,2}.objs, 2);
        [minSumInRun, idx] = min(objective_sums);

        objs = result{1,2}(1, idx).obj;
        f1   = objs(1);
        f2   = objs(2);

        droneRoutes = numel(result{1,2}(1, idx).add{1,1}{3,1});
        actSats     = sum(result{1,2}(1, idx).add{1,1}{1,1} > dep_th);

        if minSumInRun < best.score
            best.score = minSumInRun;
            best.f1 = f1; best.f2 = f2;
            best.droneRoutes = droneRoutes;
            best.actSats = actSats;
        end
    end
end

% ===================== 1) Compute baseline per problem (W=1.0, E=1.0) =====================
baseline = repmat(struct('f1',nan,'f2',nan,'sum',nan,'droneRoutes',nan,'actSats',nan), 1, num_problems);

% 找到 "DSDE_1W1E" 在 algorithm_name 中的位置
base_idx = find(strcmp(algorithm_name, 'DSDE_1W1E'), 1);
if isempty(base_idx)
    error('Cannot find baseline algorithm folder: DSDE_1W1E');
end

for j = 1:num_problems
    b = getBestAcrossRuns(algorithm_name{base_idx}, lables{j}, num_dep(j));
    baseline(j).f1 = b.f1;
    baseline(j).f2 = b.f2;
    baseline(j).sum = b.f1 + b.f2;
    baseline(j).droneRoutes = b.droneRoutes;
    baseline(j).actSats = b.actSats;
end

% ===================== 2) Build 5x5 heatmap data per problem =====================
% data_all{j} 是第 j 个 instance 的 5x5 矩阵（行=W，列=E）
data_all = cell(1, num_problems);
data_rate_all = cell(1, num_problems); % 可选：相对 baseline 的变化率

for j = 1:num_problems
    data = nan(5,5);        % f1+f2
    data_rate = nan(5,5);   % (f1+f2 - baseline)/baseline

    for i = 1:numel(algorithm_name)
        [Wval, Eval] = parseWE(string(algorithm_name{i}));

        % 映射到矩阵下标：行=W，列=E
        w_idx = find(abs(W_levels - Wval) < 1e-9, 1);
        e_idx = find(abs(E_levels - Eval) < 1e-9, 1);

        if isempty(w_idx) || isempty(e_idx)
            continue; % 防御性：命名不在 5 个等级里就跳过
        end

        s = getBestAcrossRuns(algorithm_name{i}, lables{j}, num_dep(j));
        val = s.f1 + s.f2;

        data(w_idx, e_idx) = val;
        data_rate(w_idx, e_idx) = (val - baseline(j).sum) / baseline(j).sum;
    end

    data_all{j} = data;
    data_rate_all{j} = data_rate;
end

% ===================== 3) Plot heatmap for one instance (example: j=1) =====================
% ====== choose which instance to plot ======
j = 4;  % 1..4
data      = data_all{j};         % f1+f2 (absolute)
data_rate = data_rate_all{j};    % relative change to baseline

% baseline cell (W=1, E=1) must be (3,3)
data_rate(3,3) = 0;

% ====== plot: heatmap of relative change (%) ======
Z = 100 * data_rate;  % convert to percentage

[X, Y] = meshgrid(battery_capacity_variations, payload_capacity_variations);

figure;
imagesc(X(1,:), Y(:,1), Z);
set(gca, 'YDir', 'normal');  % -20% bottom, +20% top
colormap('cool');
cb = colorbar;
cb.Label.String = 'Relative change of (f_1+f_2) (%)';

xlabel('Drone endurance variation (%)'); % E
ylabel('Drone payload variation (%)');   % W
title(['Relative change heatmap (', strrep(lables{j}, '_', '\_'), ')']);

xticks(battery_capacity_variations);
yticks(payload_capacity_variations);
axis tight;

% ====== make color contrast clearer but not "too dramatic" ======
% Use a robust symmetric range around 0 (trim extremes).
vec = Z(~isnan(Z));
vec = vec(:);
vec = sort(vec);

if isempty(vec)
    error('Z is empty (all NaN). Check data filling.');
end

% 90% central range (trim 5% on each side)
n = numel(vec);
lo = vec(max(1, round(0.05*n)));
hi = vec(min(n, round(0.95*n)));
maxAbs = max(abs([lo, hi]));

% Keep a minimum range to avoid over-amplifying tiny noise
minRange = 1.0;   % 1% minimum
maxAbs = max(maxAbs, minRange);

% Optional: cap range so it doesn't look too "significant"
capRange = 25;    % show at most +/-25%
maxAbs = min(maxAbs, capRange);

caxis([-maxAbs, maxAbs]);

% ====== dashed separators between blocks ======
% cell centers are at [-20,-10,0,10,20], boundaries at midpoints
x_mid = (battery_capacity_variations(1:end-1) + battery_capacity_variations(2:end))/2;
y_mid = (payload_capacity_variations(1:end-1) + payload_capacity_variations(2:end))/2;

hold on;
for k = 1:numel(x_mid)
    xline(x_mid(k), '--', 'LineWidth', 0.8);
end
for k = 1:numel(y_mid)
    yline(y_mid(k), '--', 'LineWidth', 0.8);
end

% ====== overlay percentage text inside each cell ======
for r = 1:5
    for c = 1:5
        if isnan(Z(r,c)); continue; end

        x = battery_capacity_variations(c);
        y = payload_capacity_variations(r);

        if r==3 && c==3
            txt = sprintf('%.1f%%\n(base)', Z(r,c));
        else
            txt = sprintf('%+.1f%%', Z(r,c));
        end

        text(x, y, txt, 'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', 'FontSize', 9, 'FontWeight','bold', ...
    'Color','w');

    end
end
hold off;
