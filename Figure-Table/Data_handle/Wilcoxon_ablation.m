% 给定的维度
num_problems = 60;  % 问题数量
num_algorithms = 4;  % 算法数量
num_runs = 8;  % 每个算法每个问题的运行次数
algorithm_name = {'DSDE_old','DSDE_noEp','noDSDE','DSDE'};
lables = {'Ca1-2,3,15','Ca1-3,5,15','Ca1-6,4,15','Ca2-2,3,15','Ca2-3,5,15','Ca2-6,4,15',...
        'Ca3-2,3,15','Ca3-3,5,15','Ca3-6,4,15','Ca4-2,3,15','Ca4-3,5,15'...
        'Ca4-6,4,15','Ca5-2,3,15','Ca5-3,5,15','Ca5-6,4,15','Ca1-2,3,30','Ca1-3,5,30','Ca1-6,4,30','Ca2-2,3,30','Ca2-3,5,30','Ca2-6,4,30',...
        'Ca3-2,3,30','Ca3-3,5,30','Ca3-6,4,30','Ca4-2,3,30','Ca4-3,5,30'...
        'Ca4-6,4,30','Ca5-2,3,30','Ca5-3,5,30','Ca5-6,4,30','Ca1-2,3,50','Ca1-3,5,50','Ca1-6,4,50','Ca2-2,3,50','Ca2-3,5,50','Ca2-6,4,50',...
        'Ca3-2,3,50','Ca3-3,5,50','Ca3-6,4,50','Ca4-2,3,50','Ca4-3,5,50'...
        'Ca4-6,4,50','Ca5-2,3,50','Ca5-3,5,50','Ca5-6,4,50','Ca1-2,3,100','Ca1-3,5,100','Ca1-6,4,100','Ca2-2,3,100','Ca2-3,5,100','Ca2-6,4,100',...
        'Ca3-2,3,100','Ca3-3,5,100','Ca3-6,4,100','Ca4-2,3,100','Ca4-3,5,100'...
        'Ca4-6,4,100','Ca5-2,3,100','Ca5-3,5,100','Ca5-6,4,100'};
nadir_point = [30,4;50,6;70,8;90,10];
% 初始化按问题存储的 IGD_data 矩阵
HV_data = zeros(num_runs, num_algorithms, num_problems);
Data = cell(60,4);  

DS  = {'23','35','64'};
Cus = {'15','30','50','100'};   % 如果后面还要 150/200/... 直接往后加

for i = 1:4  % 算法
    for m = 1:numel(Cus)    % 客户规模（最外层，保证先 15 再 30 再 50 再 100）
        for j = 1:5         % Ca1~Ca5
            for k = 1:numel(DS)  % 仓库-卫星规模 23/35/64
                row = (m-1)*15 + (j-1)*3 + k;   % 关键：严格匹配你的逐行顺序
                HV_sigle = zeros(1,8);
                for n = 1:8  % 运行次数
                    filename = ['F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data1\', ...
                                algorithm_name{i},'\','DSDE_Ca',num2str(j),'_',DS{k},'_',Cus{m}, ...
                                '_TW_M2_D10_',num2str(n),'.mat'];
                    load(filename);  % 需确保文件里有 result
                    if isnan(metric.HV)
                        HV_sigle(1,n) = 0;
                    else
                        HV_sigle(1,n) = metric.HV;
                    end
                end
                Data{row, i} = HV_sigle;
            end
        end
    end
end

% 计算每个问题上各算法的均值、标准差及比较结果
results = cell(61, 5);  % 61行，9列（包括表头）

% 表头：问题名称与算法名
results{1, 1} = 'Problem';
results{1, 2} = algorithm_name{1};
results{1, 3} = algorithm_name{2};
results{1, 4} = algorithm_name{3};
results{1, 5} = algorithm_name{4};
% results{1, 6} = algorithm_name{5};
% results{1, 7} = algorithm_name{6};

epsilon = 1e-4;  % 设置容忍度（可根据需要调整）
% 初始化统计结果
stats = zeros(3, 3);  % 每个算法的统计结果，4行（算法数），3列（领先、弱于、近似）
% 逐问题处理
% filename1 = 'F:\Onedrive\Experiment\2EVRPLD\Gurobi Result\Set1_results.xlsx';
% table_data = readtable(filename1, 'HeaderLines', 1);
% exact_results = table_data{:, 5};  % 第5列是点的属性
gap = zeros(60,4);
Data = Data';
for i = 1:60
    results{i+1, 1} = lables{i};  % 问题名称
    % 计算均值与标准差，并进行Wilcoxon检验
    comp_1 = compare(Data{1,i}, Data{4,i}, epsilon);
    comp_2 = compare(Data{2,i}, Data{4,i}, epsilon);
    comp_3 = compare(Data{3,i}, Data{4,i}, epsilon);
    % 存储计算结果
    results{i+1, 2} = sprintf('%.4e (%.4e) %s', mean(Data{1,i}), std(Data{1,i}), comp_1);
    results{i+1, 3} = sprintf('%.4e (%.4e) %s', mean(Data{2,i}), std(Data{2,i}), comp_2);
    results{i+1, 4} = sprintf('%.4e (%.4e) %s', mean(Data{3,i}), std(Data{3,i}), comp_3);
    results{i+1, 5} = sprintf('%.4e (%.4e) ', mean(Data{4,i}), std(Data{4,i}));  % CRCEA与自己比较，始终为"="
    
    % for j = 1:6
    %     gap(i,j) = (abs(min(Data{j,i})-exact_results(i))./exact_results(i));
    % end

    % 统计每个算法与DSDE的比较结果
    if strcmp(comp_1, '+')
        stats(1, 1) = stats(1, 1) + 1;  % LNS领先
    elseif strcmp(comp_1, '-')
        stats(1, 2) = stats(1, 2) + 1;  % LNS弱于
    else
        stats(1, 3) = stats(1, 3) + 1;  % LNS近似
    end
    if strcmp(comp_2, '+')
        stats(2, 1) = stats(2, 1) + 1;  % ALNS领先
    elseif strcmp(comp_2, '-')
        stats(2, 2) = stats(2, 2) + 1;  % ALNS弱于
    else
        stats(2, 3) = stats(2, 3) + 1;  % ALNS近似
    end
    if strcmp(comp_3, '+')
        stats(3, 1) = stats(3, 1) + 1;  % CMOEAD领先
    elseif strcmp(comp_3, '-')
        stats(3, 2) = stats(3, 2) + 1;  % CMOEAD弱于
    else
        stats(3, 3) = stats(3, 3) + 1;  % CMOEAD近似
    end
end
% 添加统计结果行到表格
results{62, 1} = '+,-,=';  % 第一列：'+,-,='
results{62, 2} = sprintf('%d/%d/%d', stats(1, 1), stats(1, 2), stats(1, 3));  % LNS统计结果
results{62, 3} = sprintf('%d/%d/%d', stats(2, 1), stats(2, 2), stats(2, 3));  % ALNS统计结果
results{62, 4} = sprintf('%d/%d/%d', stats(3, 1), stats(3, 2), stats(3, 3));  % CMOEAD统计结果

% 将结果写入Excel文件
xlswrite('Compare_ablation.xlsx', results);
gap(abs(gap) < 0.001) = 0;           % 小于0.01的置为0
gap = round(gap, 4);

% xlswrite('Gap_8.xlsx', gap);
% disp('Results have been written to algorithm_comparison_results.xlsx');


% 函数：比较每个算法与CRCEA的差异（使用Wilcoxon秩和检验）
function comparison = compare(x, y, epsilon)
    % 进行Wilcoxon秩和检验，比较x和y
    [~, h] = signrank(x, y);  % 对比 x 和 y 的配对样本
%     如果p值大于显著性水平（0.05），认为两者差异不显著，返回“=”
    if ~h
        comparison = '=';
    else
        % 如果p值小于显著性水平，进一步检查均值的差异
        mean_x = mean(x);
        mean_y = mean(y);
        % 容忍度：如果两者均值的差异小于epsilon，则认为两者差异不显著
        if abs(mean_x - mean_y) < mean([mean_x,mean_y])*epsilon
            comparison = '='; % 认为它们相等
        elseif mean_x > mean_y
            comparison = '+';  % 如果x的均值大于y，返回"+"
        else
            comparison = '-';  % 如果x的均值小于y，返回"-"
        end
    end
end
