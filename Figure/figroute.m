
clc;
clear;

% 更显眼的颜色组合（从绿到红）
color = [0/255, 255/255, 0/255;        % 亮绿色（高电量）
         255/255, 255/255, 0/255;      % 亮黄色（75%电量）
         255/255, 150/255, 0/255;      % 橙色（50%电量）
         255/255, 80/255, 0/255;       % 深橙色（25%电量）
         255/255, 0/255, 0/255];       % 红色（低电量）

nColors = 1000; % 定义渐变颜色的数量

% 创建非线性映射：前50%电池对应80%颜色变化，后50%对应20%颜色变化
battery_level = linspace(0, 1, nColors)';
color_pos = zeros(nColors, 1);

for i = 1:nColors
    if battery_level(i) <= 0.5
        color_pos(i) = 0.7 * (battery_level(i)/0.5); % 前50%→80%颜色
    else
        color_pos(i) = 0.7 + 0.3 * ((battery_level(i)-0.5)/0.5); % 后50%→20%颜色
    end
end

% 插值生成颜色映射
color_nodes = linspace(0, 1, size(color, 1));
colors = zeros(nColors, 3);
for c = 1:3
    colors(:, c) = interp1(color_nodes, color(:, c), color_pos, 'linear');
end

% Energy 的标尺是从 1 到 0
Energy_scale = linspace(1, 0, nColors); % Energy 从 1 到 0
result = 0;

algorithm_name =  {'CMOEAD','PPS','DNNSGAII','CMMO','SFEA','DSDE'};
num_set = {'Ca1','Ca2','Ca3','Ca4','Ca5'};
num_depsat = {'23','35','64'};
[vrp2e,~,~,~,~,~,~,~,~,~] = extractdata(1,10);
Population = [];
for i = 1:8
    filePath = ['F:\Onedrive\PlatEMO-2EVRPLDTW\Data\',algorithm_name{6},'\',algorithm_name{6},'_',num_set{1},'_',num_depsat{3},'_30_TW_M2_D30_',num2str(i),'.mat'];
    disp(filePath);  % 输出生成的文件路径，检查是否正确
    load(filePath);
    Pop = result{1,2};
%     Pop = ArchiveUpdate(Pop,100);
    Population = [Population,Pop];
end
    Population = ArchiveUpdate(Population,100);

    opt_sat = Population(1).add{1,1}{1,1}{1,1};
    opt_cus = Population(1).add{1,1}{1,1}{2,1};
    coord_dep = vrp2e.coord_dep;
    coord_sat = vrp2e.coord_sat;
    coord_cus = vrp2e.coord_cus;
    type = vrp2e.type;
    dis_sc = vrp2e.dis_sc;
    E = vrp2e.E;
    V = vrp2e.V;
%     vlt = find(opt_vlts==0);
%     vrp = find(opt_vrps==min(opt_vrps(vlt)),1);
%     result = opt_vrps(vrp);
%     opt_sat = opt_sats{vrp};
%     opt_cus = opt_cuss{vrp};
%     opt_sat(opt_sat(1:1:end) == 0)=[];
%     opt_cus(opt_cus(1:1:end) == 0)=[];
    figure;
    trans = 1;  %标注的位置平移量
    plot(coord_dep(:,1),coord_dep(:,2),'s','MarkerEdgeColor','b','MarkerFaceColor','r','MarkerSize',10);
    hold on;
    plot(coord_sat(:,1),coord_sat(:,2),'h','MarkerEdgeColor','r','MarkerFaceColor','b','MarkerSize',10);
    hold on;
    for i = 1:size(coord_cus,1)
        if strcmp(type{i}, 'delivery')
            plot(coord_cus(i,1),coord_cus(i,2),'o','MarkerEdgeColor','none','MarkerFaceColor',[0.5, 0.5, 0.5],'MarkerSize',8);
        else
            plot(coord_cus(i,1),coord_cus(i,2),'o','MarkerEdgeColor', [0.5, 0.5, 0.5], 'MarkerFaceColor', 'none', 'LineWidth', 1,'MarkerSize',8);
        end
    end
    
    hold on;

    %----画出第一层路径----
    len_sat = length(opt_sat);
    xy_sat  = zeros(len_sat,2);
    label_s   = cell(1,len_sat);
    k1 = 1;         %记录存储矩阵的位置
    k2 = 1;         %记录卫星点
    xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
    label_s{k1}  = opt_sat(k2);
    for i = 2:len_sat
        k1 = k1+1;
        if opt_sat(i)<=length(coord_dep)
            xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
            label_s{k1}  = opt_sat(k2);
            plot(xy_sat(1:k1,1),xy_sat(1:k1,2),'--b');
%             text(xy_cus(2:r1-1,1)+trans,xy_cus(2:r1-1,2)+trans, label_c(2:r1-1));
            hold on;
            k2 = i;
            k1 = 1;
            xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
            label_s{k1}  = opt_sat(k2);
        else
            xy_sat(k1,:) = coord_sat(opt_sat(i)-length(coord_dep),:);
            label_s{k1}  = opt_sat(i);
        end
    end
    k1 = k1+1;
    xy_sat(k1,:) = coord_dep(opt_sat(k2),:);
    label_s{k1}  = opt_sat(k2);
    plot(xy_sat(1:k1,1),xy_sat(1:k1,2),'--b');
%     text(xy_sat(:,1)+trans,xy_sat(:,2)+trans, label_s);
    hold on;
    
    %----画出第二层路径----
    len_cus = length(opt_cus);
    xy_cus  = zeros(len_cus,2);
    label_c = cell(1,len_cus);
    r1 = 1;         %记录存储矩阵的位置
    r2 = 1;         %记录卫星点
    xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
    label_c{r1}  = opt_cus(r2);
    for i = 2:len_cus
        r1 = r1+1;
        if opt_cus(i)<=length(coord_sat)
            xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
            label_c{r1}  = opt_cus(r2);
            route = [opt_cus(r2:i-1),opt_cus(r2)];  %子路段
            distance_route = dis_sc(sub2ind(size(dis_sc), route(1:end-1), route(2:end))); %子路段长度
            route_cap = energy(vrp2e,route);   %每个路段的载重
            Energy_cap = cell2mat(arrayfun(@(m,n)sum((5+m)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(n/V)), route_cap,distance_route, 'UniformOutput', false)); 
            Energy = (E - [0, cumsum(Energy_cap)])/E;
            for k =1:length(route_cap)
                Energy_A = Energy(k);
                Energy_B = Energy(k+1);
                [~, idx_A] = min(abs(Energy_scale - Energy_A));  % 找到 Energy_A 对应的颜色位置
                [~, idx_B] = min(abs(Energy_scale - Energy_B));  % 找到 Energy_B 对应的颜色位置
                num = idx_B-idx_A;
                % 计算渐变直线的坐标
                x = linspace(xy_cus(k,1), xy_cus(k+1,1), num);  % num 个点的 x 坐标
                y = linspace(xy_cus(k,2), xy_cus(k+1,2), num);  % num 个点的 y 坐标
                for p = 1:length(x)-1
                    % 使用 Energy 标尺的颜色
                    plot(x(p:p+1), y(p:p+1), 'Color', colors(idx_A+p, :), 'LineWidth', 3);
                end
                plot(xy_cus(k:k+1,1),xy_cus(k:k+1,2), 'LineStyle', '--', 'LineWidth', 1.5,'Color', color(ceil(route_cap(k)/2+0.01), :));
            end
%             text(xy_cus(2:r1-1,1)+trans,xy_cus(2:r1-1,2)+trans, label_c(2:r1-1));
            hold on;
            r2 = i;
            r1 = 1;
            xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
            label_c{r1}  = opt_cus(r2);
        else
            xy_cus(r1,:) = coord_cus(opt_cus(i)-length(coord_sat),:);
            label_c{r1}  = opt_cus(i);
        end
    end
    r1 = r1+1;
    xy_cus(r1,:) = coord_sat(opt_cus(r2),:);
    label_c{r1}  = opt_cus(r2);
    route = [opt_cus(r2:end),opt_cus(r2)];  %子路段
    distance_route = dis_sc(sub2ind(size(dis_sc), route(1:end-1), route(2:end))); %子路段长度
    route_cap = energy(vrp2e,route);   %每个路段的载重
    Energy_cap = cell2mat(arrayfun(@(m,n)sum((5+m)*(3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4).*(n/V)), route_cap,distance_route, 'UniformOutput', false)); 
    Energy = (E - [0, cumsum(Energy_cap)])/E;
    for k =1:length(route_cap)
        Energy_A = Energy(k);
        Energy_B = Energy(k+1);
        [~, idx_A] = min(abs(Energy_scale - Energy_A));  % 找到 Energy_A 对应的颜色位置
        [~, idx_B] = min(abs(Energy_scale - Energy_B));  % 找到 Energy_B 对应的颜色位置
        num = idx_B-idx_A;
        % 计算渐变直线的坐标
        x = linspace(xy_cus(k,1), xy_cus(k+1,1), num);  % 100 个点的 x 坐标
        y = linspace(xy_cus(k,2), xy_cus(k+1,2), num);  % 100 个点的 y 坐标
        for p = 1:length(x)-1
            % 使用 Energy 标尺的颜色
            plot(x(p:p+1), y(p:p+1), 'Color', colors(idx_A+p, :), 'LineWidth', 3);
        end
        plot(xy_cus(k:k+1,1),xy_cus(k:k+1,2), 'LineStyle', '--', 'LineWidth', 1.5,'Color', color(ceil(route_cap(k)/2+0.01), :));
    end
%     text(xy_cus(2:r1-1,1)+trans,xy_cus(2:r1-1,2)+trans, label_c(2:r1-1));
    hold off;
    xlabel('x-coordinate','FontWeight','bold');
    ylabel('y-coordinate','FontWeight','bold');
%     title(['Best cost',num2str(result)],'FontWeight','bold');
%     legend('Even Design',1);
    axis equal;
    grid on;
    box on;

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