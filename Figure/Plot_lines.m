algorithms = {'CMOEAD','PPS','DNNSGAII','CMMO','SFEA','DSDE'};
A = zeros(6,200);
for i = 1:length(algorithms)
    algo_name = algorithms{i};
    load(['F:\Onedrive\Experiment\2EVRPLDTW\Cov\',algo_name,'_Ca1_35_100_TW_HV.mat'])
    A(i,:) = I1;
end

% 假设数据
generations = 1:200;  
HV_values = A'; % 替换为你的实际数据

% 1. 定义配色和形状
colors = [
    [0.0, 0.4470, 0.7410]    % 深蓝（CMOEAD）
    [0.8500, 0.3250, 0.0980] % 橙红（PPS）
    [0.4660, 0.6740, 0.1880] % 草绿（DNNSGAII）
    [0.4940, 0.1840, 0.5560] % 紫（CMMO）
    [0.9290, 0.6940, 0.1250] % 金黄（SFEA）
    [0.3010, 0.7450, 0.9330] % 天蓝（DSDN）
];


markers = {'o', 's', 'd', '^', 'v', 'p'}; % 6种不同标记

% 2. 绘制图形
figure;
hold on;
grid on;
box on;

% 循环绘制每个算法（点线结合）
for i = 1:6
    plot(generations, HV_values(:, i), ...
        'Color', colors(i, :),'LineWidth', 1.2,'Marker', markers{i},'MarkerIndices', 1:10:200,'MarkerSize', 6, 'MarkerFaceColor', colors(i, :));
        % 线条颜色           % 线宽               % 点形状       % 每隔10代显示一个点    % 点大小   % 点填充颜色
end

% 3. 添加图例和标签
legend('CMOEAD', 'PPS', 'DNNSGAII', 'CMMO', 'SFEA', 'DSDE', ...
    'Location', 'northwest', 'FontSize', 10);
xlabel('Generation', 'FontSize', 12);
ylabel('Hypervolume (HV)', 'FontSize', 12);
% title('HV Trends with Point-Line Combination', 'FontSize', 14);
set(gca, 'FontName', 'Arial', 'FontSize', 11);
hold off;

