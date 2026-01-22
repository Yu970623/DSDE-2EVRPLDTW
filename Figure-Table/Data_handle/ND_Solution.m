algorithms = {'LNS','ALNS','CMOEAD','DNNSGAII','SFEA','DSDE'};
Compare_algo = cell(1,length(algorithms));

for i = 1:2  % 算法
    Population = [];
    for j = 51 % 注意：现在遍历 1..60
        for k = 1:8  
            filename = ['F:\Onedrive\Experiment\', algorithms{i}, '\', ...
                        algorithms{i}, '_Set1_num', num2str(j), '_NDarchive.mat'];
            load(filename)            % 需要包含 nd_objs
            Population = [Population; nd_objs{k}];
        end
    end
        % 先精确去重
    Compare_algo{i} = unique(Population, 'rows');
    
    % 二次去重（基于行和差异）
    [~, idx] = unique(round(sum(Compare_algo{i}, 2) * 10));
    Compare_algo{i} = Compare_algo{i}(sort(idx), :);
end

for i = 3:6
    Population = [];
    for j = 1:8
        algo_name = algorithms{i};
        load(['F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data1\',algo_name,'\',algo_name,'_Ca5_23_50_TW_M2_D10_',num2str(j),'.mat'])
        Pop = result{1,2};
        Population = [Population,Pop];
    end
    Population = ArchiveUpdate(Population,100);
    Compare_algo{i} = unique(Population.objs, 'rows', 'stable');
end

% 1. 定义配色和形状
colors = [
    [0.0, 0.4470, 0.7410]    % 深蓝（CMOEAD）
    [0.8500, 0.3250, 0.0980] % 橙红（PPS）
    [0.4660, 0.6740, 0.1880] % 草绿（DNNSGAII）
    [0.4940, 0.1840, 0.5560] % 紫（CMMO）
    [0.9290, 0.6940, 0.1250] % 金黄（SFEA）
    [0.3010, 0.7450, 0.9330] % 天蓝（DSDN）
];

figure
scatter(Compare_algo{1}(:,1),Compare_algo{1}(:,2),30,'o','Markerfacecolor',colors(1,:),'Markeredgecolor',colors(1,:))
hold on
scatter(Compare_algo{2}(:,1),Compare_algo{2}(:,2), 30,'o','Markerfacecolor',colors(2,:),'Markeredgecolor',colors(2,:))

scatter(Compare_algo{3}(:,1),Compare_algo{3}(:,2), 30,'o','Markerfacecolor',colors(3,:),'Markeredgecolor',colors(3,:))

scatter(Compare_algo{4}(:,1),Compare_algo{4}(:,2), 30,'o','Markerfacecolor',colors(4,:),'Markeredgecolor',colors(4,:))

scatter(Compare_algo{5}(:,1),Compare_algo{5}(:,2), 30,'o','Markerfacecolor',colors(5,:),'Markeredgecolor',colors(5,:))

scatter(Compare_algo{6}(:,1),Compare_algo{6}(:,2), 30,'o','Markerfacecolor',colors(6,:),'Markeredgecolor',colors(6,:))

legend('LNS','ALNS','CMOEAD','DNNSGAII','SFEA','DSDE', 'Location', 'northeast');
xlabel('f1');
ylabel('f2');

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