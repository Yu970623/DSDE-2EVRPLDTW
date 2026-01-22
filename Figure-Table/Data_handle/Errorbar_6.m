% 给定的维度
num_problems = 15;  % 问题数量
num_algorithms = 6;  % 算法数量
num_runs = 8;  % 每个算法每个问题的运行次数
algorithm_name = {'LNS','ALNS','CMOEAD','DNNSGAII','SFEA','DSDE'};
nadir_point = [30,4;50,6;70,8;90,10];
% 初始化按问题存储的 IGD_data 矩阵
HV_data = zeros(num_runs, num_algorithms, num_problems);
Data = cell(15,6);  
for i = 1:2  % 算法
    HV_sigle = zeros(1,8);
    for j = 1:4:60  % 注意：现在遍历 1..60，靠 row 映射到 1..60 行
        % 计算要填充到 Data 的行号（1-15, 16-30, 31-45, 46-60）
        % g   = mod(j-1, 4) + 1;        % 组号: 1..4  (对应 j=1/2/3/4:4:60)
        % t   = floor((j-1)/4) + 1;     % 组内位置: 1..15
        % row = (g-1)*15 + t;           % 总行号: 1..60

        HV_sigle = zeros(1,8);        % 每个 j 单独算一行（8次重复） 
        for k = 1:8  
            filename = ['F:\Onedrive\Experiment\', algorithm_name{i}, '\', ...
                        algorithm_name{i}, '_Set1_num', num2str(j), '_NDarchive.mat'];
            load(filename)            % 需要包含 nd_objs
            Pop_ND = nd_objs{k};
            if isnan(HV_out(Pop_ND, nadir_point(1,:)))
                HV_sigle(1,k) = 0;
            else
                HV_sigle(1,k) = HV_out(Pop_ND, nadir_point(1,:));
            end
        end
        Data{ceil(j/4), i} = HV_sigle;
    end
end
DS  = {'23','35','64'};
Cus = {'15','30','50','100'};   % 如果后面还要 150/200/... 直接往后加

time_DSDE = zeros(1,30);
for i = 6  % 算法
    for m = 1:2  % 客户规模（最外层，保证先 15 再 30 再 50 再 100）
        for j = 1:5         % Ca1~Ca5
            for k = 1:numel(DS)  % 仓库-卫星规模 23/35/64
                row = (j-1)*3 + k;   % 关键：严格匹配你的逐行顺序
                HV_sigle = zeros(1,8);
                time_single =  zeros(1,8);
                for n = 1:8  % 运行次数
                    filename = ['F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data\', ...
                                algorithm_name{i},'\', ...
                                algorithm_name{i},'_Ca',num2str(j),'_',DS{k},'_',Cus{m}, ...
                                '_TW_M2_D10_',num2str(n),'.mat'];
                    load(filename);  % 需确保文件里有 result
                    HV_sigle(1,n) = HV_out(result{1,2}.objs, nadir_point(1,:));
                    time_single(1,n) = metric.runtime;
                end
                Data{row, i} = HV_sigle;
                time_DSDE(1,row+(row*(m-1))) = mean(time_single);
            end
        end
    end
end
time_DSDE
results = zeros(8,6);
Fred = zeros(120,6);
% 循环读取每个问题的文件
for problem = 1:num_problems
    results(:,1) = Data{problem,1};
    results(:,2) = Data{problem,2};
    results(:,3) = Data{problem,3};
    results(:,4) = Data{problem,4};
    results(:,5) = Data{problem,5};
    results(:,6) = Data{problem,6};
    HV_data(:, :, problem) = results;
    Fred((problem-1)*8 + 1:problem*8, :) = results;
end
% writematrix(Fred, 'D:\OneDrive\OneDrive - csu.edu.cn\2E-VRP\Experiment\drawNemenyi-master\Data\6_15.xlsx');
% 计算平均 IGD 和标准误差
mean_IGD_data = mean(HV_data, 1);  % 计算每个算法在所有问题上的平均 IGD，结果为 1 x num_algorithms x num_problems
std_error_data = std(HV_data, 0, 1) / sqrt(num_runs);  % 计算每个算法在所有问题上的标准误差

% 算法颜色设置
colors = [lines(7); [1, 0, 0]];  % MATLAB 默认颜色序列

% 绘图
figure;
hold on;

% 循环绘制每个算法的平均 IGD 和置信区间
for alg = 1:num_algorithms
    % 绘制平均 HV
    h_line = plot(1:num_problems, squeeze(mean_IGD_data(1, alg, :)), '-o', 'Color', colors(alg, :), 'MarkerSize', 5);
    
    % 使用 t 分布计算置信区间
    t_critical = tinv(0.9, num_runs - 1);  % 95% 置信水平
    error_range = squeeze(t_critical * std_error_data(1, alg, :));  % 置信区间
    
    % 绘制置信区间
    for i = 1:num_problems
        % 上下界
        line([i, i], [squeeze(mean_IGD_data(1, alg, i)) - error_range(i), squeeze(mean_IGD_data(1, alg, i)) + error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
        % 横线
        plot([i-0.1, i+0.1], [squeeze(mean_IGD_data(1, alg, i)) - error_range(i), squeeze(mean_IGD_data(1, alg, i)) - error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
        plot([i-0.1, i+0.1], [squeeze(mean_IGD_data(1, alg, i)) + error_range(i), squeeze(mean_IGD_data(1, alg, i)) + error_range(i)], ...
            'Color', colors(alg, :), 'LineWidth', 2);
    end
end
labels15 = {'Ca1-2,3,15','Ca1-3,5,15','Ca1-6,4,15','Ca2-2,3,15','Ca2-3,5,15','Ca2-6,4,15','Ca3-2,3,15',...
        'Ca3-3,5,15','Ca3-6,4,15','Ca4-2,3,15','Ca4-3,5,15','Ca4-6,4,15','Ca5-2,3,15','Ca5-3,5,15','Ca5-6,4,15', ...
        'Cb1-2,3,15','Cb1-3,5,15','Cb1-6,4,15','Cb2-2,3,15','Cb2-3,5,15','Cb2-6,4,15','Cb3-2,3,15',...
        'Cb3-3,5,15','Cb3-6,4,15','Cb4-2,3,15','Cb4-3,5,15','Cb4-6,4,15','Cb5-2,3,15','Cb5-3,5,15','Cb5-6,4,15', ...
        'Cc1-2,3,15','Cc1-3,5,15','Cc1-6,4,15','Cc2-2,3,15','Cc2-3,5,15','Cc2-6,4,15','Cc3-2,3,15',...
        'Cc3-3,5,15','Cc3-6,4,15','Cc4-2,3,15','Cc4-3,5,15','Cc4-6,4,15','Cc5-2,3,15','Cc5-3,5,15','Cc5-6,4,15', ...
        'Cd1-2,3,15','Cd1-3,5,15','Cd1-6,4,15','Cd2-2,3,15','Cd2-3,5,15','Cd2-6,4,15','Cd3-2,3,15',...
        'Cd3-3,5,15','Cd3-6,4,15','Cd4-2,3,15','Cd4-3,5,15','Cd4-6,4,15','Cd5-2,3,15','Cd5-3,5,15','Cd5-6,4,15'};
labels30 = {'Ca1-2,3,30','Ca1-3,5,30','Ca1-6,4,30','Ca2-2,3,30','Ca2-3,5,30','Ca2-6,4,30','Ca3-2,3,30',...
        'Ca3-3,5,30','Ca3-6,4,30','Ca4-2,3,30','Ca4-3,5,30','Ca4-6,4,30','Ca5-2,3,30','Ca5-3,5,30','Ca5-6,4,30', ...
        'Cb1-2,3,30','Cb1-3,5,30','Cb1-6,4,30','Cb2-2,3,30','Cb2-3,5,30','Cb2-6,4,30','Cb3-2,3,30',...
        'Cb3-3,5,30','Cb3-6,4,30','Cb4-2,3,30','Cb4-3,5,30','Cb4-6,4,30','Cb5-2,3,30','Cb5-3,5,30','Cb5-6,4,30', ...
        'Cc1-2,3,30','Cc1-3,5,30','Cc1-6,4,30','Cc2-2,3,30','Cc2-3,5,30','Cc2-6,4,30','Cc3-2,3,30',...
        'Cc3-3,5,30','Cc3-6,4,30','Cc4-2,3,30','Cc4-3,5,30','Cc4-6,4,30','Cc5-2,3,30','Cc5-3,5,30','Cc5-6,4,30', ...
        'Cd1-2,3,30','Cd1-3,5,30','Cd1-6,4,30','Cd2-2,3,30','Cd2-3,5,30','Cd2-6,4,30','Cd3-2,3,30',...
        'Cd3-3,5,30','Cd3-6,4,30','Cd4-2,3,30','Cd4-3,5,30','Cd4-6,4,30','Cd5-2,3,30','Cd5-3,5,30','Cd5-6,4,30'};
labels50 = {'Ca1-2,3,50','Ca1-3,5,50','Ca1-6,4,50','Ca2-2,3,50','Ca2-3,5,50','Ca2-6,4,50','Ca3-2,3,50',...
        'Ca3-3,5,50','Ca3-6,4,50','Ca4-2,3,50','Ca4-3,5,50','Ca4-6,4,50','Ca5-2,3,50','Ca5-3,5,50','Ca5-6,4,50', ...
        'Cb1-2,3,50','Cb1-3,5,50','Cb1-6,4,50','Cb2-2,3,50','Cb2-3,5,50','Cb2-6,4,50','Cb3-2,3,50',...
        'Cb3-3,5,50','Cb3-6,4,50','Cb4-2,3,50','Cb4-3,5,50','Cb4-6,4,50','Cb5-2,3,50','Cb5-3,5,50','Cb5-6,4,50', ...
        'Cc1-2,3,50','Cc1-3,5,50','Cc1-6,4,50','Cc2-2,3,50','Cc2-3,5,50','Cc2-6,4,50','Cc3-2,3,50',...
        'Cc3-3,5,50','Cc3-6,4,50','Cc4-2,3,50','Cc4-3,5,50','Cc4-6,4,50','Cc5-2,3,50','Cc5-3,5,50','Cc5-6,4,50', ...
        'Cd1-2,3,50','Cd1-3,5,50','Cd1-6,4,50','Cd2-2,3,50','Cd2-3,5,50','Cd2-6,4,50','Cd3-2,3,50',...
        'Cd3-3,5,50','Cd3-6,4,50','Cd4-2,3,50','Cd4-3,5,50','Cd4-6,4,50','Cd5-2,3,50','Cd5-3,5,50','Cd5-6,4,50'};
labels100 ={'Ca1-2,3,100','Ca1-3,5,100','Ca1-6,4,100','Ca2-2,3,100','Ca2-3,5,100','Ca2-6,4,100','Ca3-2,3,100',...
        'Ca3-3,5,100','Ca3-6,4,100','Ca4-2,3,100','Ca4-3,5,100','Ca4-6,4,100','Ca5-2,3,100','Ca5-3,5,100','Ca5-6,4,100', ...
        'Cb1-2,3,100','Cb1-3,5,100','Cb1-6,4,100','Cb2-2,3,100','Cb2-3,5,100','Cb2-6,4,100','Cb3-2,3,100',...
        'Cb3-3,5,100','Cb3-6,4,100','Cb4-2,3,100','Cb4-3,5,100','Cb4-6,4,100','Cb5-2,3,100','Cb5-3,5,100','Cb5-6,4,100', ...
        'Cc1-2,3,100','Cc1-3,5,100','Cc1-6,4,100','Cc2-2,3,100','Cc2-3,5,100','Cc2-6,4,100','Cc3-2,3,100',...
        'Cc3-3,5,100','Cc3-6,4,100','Cc4-2,3,100','Cc4-3,5,100','Cc4-6,4,100','Cc5-2,3,100','Cc5-3,5,100','Cc5-6,4,100', ...
        'Cd1-2,3,100','Cd1-3,5,100','Cd1-6,4,100','Cd2-2,3,100','Cd2-3,5,100','Cd2-6,4,100','Cd3-2,3,100',...
        'Cd3-3,5,100','Cd3-6,4,100','Cd4-2,3,100','Cd4-3,5,100','Cd4-6,4,100','Cd5-2,3,100','Cd5-3,5,100','Cd5-6,4,100'};

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
ylabel('cost');
xlim([0,16]);
ylim([-0.05,0.3]);
% ylim([1400,4000]);
% ylim([2000,5500]);
% ylim([3500,9500]);
grid on;
hold off;
set(gca,'XTick',1:1:16);
set(gca,'XTickLabel',label15);

% legend('LNS','ALNS','EHG','TPEA');


% figure
% x = 1:0.1:10;
% y = 1:8;
% colors = [lines(7); [1, 0, 0]];
% hold on
% for i = 1:8
%     plot(x, y(i)*ones(size(x)), 'Color', colors(i, :), 'LineWidth', 2)
% end
% hold off



% % 定义横坐标范围
% x = 0:0.01:5;
% 
% % 获取6种颜色
% colors = lines(5);
% 
% % 创建一个新的图形窗口
% figure;
% hold on; % 保持图形窗口，以便在同一图中绘制多条线
% 
% % 循环绘制每条横线
% for y = 1:5
%     plot(x, y * ones(size(x)), 'Color', colors(y, :), 'LineWidth', 2);
% end
% 
% % 设置图形属性
% xlabel('X轴');
% ylabel('Y轴');
% title('多条横线');
% grid on; % 显示网格
% axis([0 5 0 7]); % 设置坐标轴范围
% hold off; % 释放图形窗口
