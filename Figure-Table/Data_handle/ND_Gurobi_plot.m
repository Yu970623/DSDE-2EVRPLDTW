algorithms = {'DSDE'};
Compare_algo = cell(1,length(algorithms));

for i = 1:length(algorithms)
    Population = [];
    for j = 1:8
        algo_name = algorithms{i};
        load(['F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data\', ...
              algo_name,'\',algo_name,'_Ca4_64_15_TW_M2_D10_',num2str(j),'.mat'])
        Pop = result{1,2};
        Population = [Population, Pop];
    end
    Population = ArchiveUpdate(Population, 100);
    Compare_algo{i} = Population.objs;
end

% ============ 读取 Gurobi 的 Excel 结果 ============
xlsx_path  = 'F:\Onedrive\Gurobi\15_Gurobi.xlsx';
T = readtable(xlsx_path, 'VariableNamingRule','preserve');

% 选择你要对比的那个实例（按你的 Instances 列内容来改）
target_instance = "Ca4-6,4,15.csv";   % <-- 改成你表里对应的名字
maskInst = (string(T.("Instances")) == target_instance);

% obj1/obj2 可能因为含有 "Unsolvable" 变成文本列，这里统一转成 double
obj1 = str2double(string(T.("obj1")(maskInst)));
obj2 = str2double(string(T.("obj2")(maskInst)));

valid = ~isnan(obj1) & ~isnan(obj2);
Gurobi_all = [obj1(valid), obj2(valid)];

% 去重（权重扫出来重复很常见）
Gurobi_all = unique(Gurobi_all, 'rows');

% 取 Gurobi 这批点的非支配前沿
if isempty(Gurobi_all)
    warning('No valid Gurobi points found for this instance. Check instance name / table columns.');
    Gurobi_ND = [];
else
    FrontNo = NDSort(Gurobi_all, 1);
    Gurobi_ND = Gurobi_all(FrontNo==1, :);
end

% ============ 绘图 ============
colors = [
    0.9290, 0.6940, 0.1250;  % 金黄：Gurobi
    0.3010, 0.7450, 0.9330;  % 天蓝：DSDE
];

figure; hold on; box on; grid on;

if ~isempty(Gurobi_ND)
    scatter(Gurobi_ND(:,1), Gurobi_ND(:,2), 60, 's', ...
        'MarkerFaceColor', colors(1,:), 'MarkerEdgeColor', colors(1,:));
end

scatter(Compare_algo{1}(:,1), Compare_algo{1}(:,2), 30, 'o', ...
    'MarkerFaceColor', colors(2,:), 'MarkerEdgeColor', colors(2,:));

legend('Gurobi', 'DSDE', 'Location', 'northwest');
xlabel('f1'); ylabel('f2');
% xlim([25,35]);
% ylim([1,2.5]);


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
