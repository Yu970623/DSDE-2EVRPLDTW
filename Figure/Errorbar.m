% 给定的维度
num_problems = 15;  % 问题数量
num_algorithms = 4;  % 算法数量
num_runs = 8;  % 每个算法每个问题的运行次数
algorithm_name =  {'DSDE_nonHDE','DSDE_nonTDC','nonDSDE','DSDE'};
% 初始化按问题存储的 IGD_data 矩阵
HV_data = zeros(num_runs, num_algorithms, num_problems);
Data = cell(4,15);
num_set = {'Ca1','Ca2','Ca3','Ca4','Ca5'};
num_depsat = {'23','35','64'};
HV_runs = zeros(1,8);

% for i = 1:num_algorithms
%     for j = 1
%         for k = 2
%             for n = 1:num_runs
%                 filename = ['F:\Onedrive\PlatEMO-2EVRPLDTW\Data\',algorithm_name{i},'\',algorithm_name{i},'_',num_set{j},'_',num_depsat{k},'_100_TW_M2_D100_',num2str(n),'.mat'];
%                 load(filename)
%                 HV = metric.HV;
%                 HV_runs(n) = HV;
%             end
%             Data{i,1} = HV_runs;
%         end
%     end
% end

% % 初始化结果矩阵
% result = zeros(1*8, 6); % 创建150行6列的零矩阵
% 
% % 遍历原始cell数组并填充结果矩阵
% for col = 1:6      % 遍历6列
%     for row = 1 % 遍历15行
%         % 计算在结果矩阵中的起始行位置
%         start_row = (row-1)*8 + 1;
%         % 获取当前cell中的10个数据
%         cell_data = Data{col, row};
%         % 将数据放入结果矩阵的相应位置
%         result(start_row:start_row+7, col) = cell_data(:);
%     end
% end
% 
% % 将结果矩阵写入Excel文件
% filename = 'F:\Onedrive\Experiment\2EVRPLDTW\Data\35100_HV.xlsx';
% writematrix(result, filename);


for i = 1:num_algorithms
    for j = 1:length(num_set)
        for k = 1:length(num_depsat)
            for n = 1:num_runs
                filename = ['F:\Onedrive\PlatEMO-2EVRPLDTW\Data\',algorithm_name{i},'\',algorithm_name{i},'_',num_set{j},'_',num_depsat{k},'_100_TW_M2_D100_',num2str(n),'.mat'];
                load(filename)
                HV = metric.HV;
                HV_runs(n) = HV;
            end
            Data{i,3*(j-1)+k} = HV_runs;
        end
    end
end

% 初始化结果矩阵
result = zeros(15*8, 4); % 创建150行6列的零矩阵

% 遍历原始cell数组并填充结果矩阵
for col = 1:4     % 遍历6列
    for row = 1:15  % 遍历15行
        % 计算在结果矩阵中的起始行位置
        start_row = (row-1)*8 + 1;
        % 获取当前cell中的10个数据
        cell_data = Data{col, row};
        % 将数据放入结果矩阵的相应位置
        result(start_row:start_row+7, col) = cell_data(:);
    end
end

% 将结果矩阵写入Excel文件
filename = 'F:\Onedrive\Experiment\2EVRPLDTW\Data\100_xiaorong_HV.xlsx';
writematrix(result, filename);


results = zeros(8,4);
Fred = zeros(800,4);
% 循环读取每个问题的文件
for problem = 1:num_problems
    results(:,1) = Data{1,problem}';
    results(:,2) = Data{2,problem}';
    results(:,3) = Data{3,problem}';
    results(:,4) = Data{4,problem}';
%     results(:,5) = Data{5,problem}';
%     results(:,6) = Data{6,problem}';
    HV_data(:, :, problem) = results;
    Fred((problem-1)*8 + 1:problem*8, :) = results;
end

% 计算平均 HV 和标准误差
mean_HV_data = mean(HV_data, 1);  % 计算每个算法在所有问题上的平均 IGD，结果为 1 x num_algorithms x num_problems
std_error_data = std(HV_data, 0, 1) / sqrt(num_runs);  % 计算每个算法在所有问题上的标准误差

% % 算法颜色设置
% colors = [
%     [0.0, 0.4470, 0.7410]    % 深蓝（CMOEAD）
%     [0.8500, 0.3250, 0.0980] % 橙红（PPS）
%     [0.4660, 0.6740, 0.1880] % 草绿（DNNSGAII）
%     [0.4940, 0.1840, 0.5560] % 紫（CMMO）
%     [0.9290, 0.6940, 0.1250] % 金黄（SFEA）
%     [0.3010, 0.7450, 0.9330] % 天蓝（DSDN）
% ];

% 算法颜色设置
colors = [
    [0.8500, 0.3250, 0.0980] % 橙红（nonHDE）
    [0.4660, 0.6740, 0.1880] % 草绿（nonTDC）
    [0.9290, 0.6940, 0.1250] % 金黄（nonDSDE）
    [0.3010, 0.7450, 0.9330] % 天蓝（DSDE）
];
% 绘图
figure;
hold on;

% 循环绘制每个算法的平均 IGD 和置信区间
for alg = 1:num_algorithms
    % 绘制平均 HV
    h_line = plot(1:num_problems, squeeze(mean_HV_data(1, alg, :)), '-o', 'Color', colors(alg, :), 'MarkerSize', 5);
    
    % 使用 t 分布计算置信区间
    t_critical = tinv(0.8, num_runs - 1);  % 90% 置信水平
    error_range = squeeze(t_critical * std_error_data(1, alg, :));  % 置信区间
    
    % 绘制置信区间
    for i = 1:num_problems
        % 上下界
        line([i, i], [squeeze(mean_HV_data(1, alg, i)) - error_range(i), squeeze(mean_HV_data(1, alg, i)) + error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
        % 横线
        plot([i-0.1, i+0.1], [squeeze(mean_HV_data(1, alg, i)) - error_range(i), squeeze(mean_HV_data(1, alg, i)) - error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
        plot([i-0.1, i+0.1], [squeeze(mean_HV_data(1, alg, i)) + error_range(i), squeeze(mean_HV_data(1, alg, i)) + error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
    end
end



label15 = {'Ca1-2,3,15','Ca1-3,5,15','Ca1-6,4,15','Ca2-2,3,15','Ca2-3,5,15','Ca2-6,4,15',...
        'Ca3-2,3,15','Ca3-3,5,15','Ca3-6,4,15','Ca4-2,3,15','Ca4-3,5,15'...
        'Ca4-6,4,15','Ca5-2,3,15','Ca5-3,5,15','Ca5-6,4,15'};
label30 = {'Ca1-2,3,30','Ca1-3,5,30','Ca1-6,4,30','Ca2-2,3,30','Ca2-3,5,30','Ca2-6,4,30',...
        'Ca3-2,3,30','Ca3-3,5,30','Ca3-6,4,30','Ca4-2,3,30','Ca4-3,5,30'...
        'Ca4-6,4,30','Ca5-2,3,30','Ca5-3,5,30','Ca5-6,4,30'};
label50 = {'Ca1-2,3,50','Ca1-3,5,50','Ca1-6,4,50','Ca2-2,3,50','Ca2-3,5,50','Ca2-6,4,50',...
        'Ca3-2,3,50','Ca3-3,5,50','Ca3-6,4,50','Ca4-2,3,50','Ca4-3,5,50'...
        'Ca4-6,4,50','Ca5-2,3,50','Ca5-3,5,50','Ca5-6,4,50'};
label100 = {'Ca1-2,3,100','Ca1-3,5,100','Ca1-6,4,100','Ca2-2,3,100','Ca2-3,5,100','Ca2-6,4,100',...
        'Ca3-2,3,100','Ca3-3,5,100','Ca3-6,4,100','Ca4-2,3,100','Ca4-3,5,100'...
        'Ca4-6,4,100','Ca5-2,3,100','Ca5-3,5,100','Ca5-6,4,100'};

% 自定义图形属性
ylabel('HV Metric');
xlim([0,16]);
ylim([0.05,0.38]);
% ylim([1400,4000]);
% ylim([2000,5500]);
% ylim([3500,9500]);
grid on;
hold off;
set(gca,'XTick',1:1:15);
set(gca,'XTickLabel',label100);

