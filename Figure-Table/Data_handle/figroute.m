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
        color_pos(i) = 0.6 * (battery_level(i)/0.5); % 前50%→80%颜色
    else
        color_pos(i) = 0.6 + 0.4 * ((battery_level(i)-0.5)/0.5); % 后50%→20%颜色
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
%% 地图数据
[vrp2e,~,~,~,~,~,~,~,~,~] = extractdata(1,8);
coord_dep = vrp2e.coord_dep;
coord_sat = vrp2e.coord_sat;
coord_cus = vrp2e.coord_cus;
num_sat = vrp2e.num_sat;
type = vrp2e.type;
dis_sc = vrp2e.dis_sc;
E = vrp2e.E;
V = vrp2e.V;


algorithm_name =  {'LNS','ALNS','CMOEAD','DNNSGAII','SFEA','DSDE'};
num_set = {'Ca1','Ca2','Ca3','Ca4','Ca5'};
num_depsat = {'23','35','64'};
al_i = 1;
Population = [];
if al_i<=2
    %对于LNS和ALNS算法
    for i = al_i  % 算法
        for j = 8  % 问题 1..60
            filename = ['F:\Onedrive\Experiment\', algorithm_name{i}, '\vrp_data\', ...
                            algorithm_name{i}, '_Set1_num', num2str(j), '.mat'];
                load(filename)
            objs = opt_vrps(1:8);
        end
    end
    [~, index] = min(objs);
    opt_sat = opt_sats{index};
    opt_cus = opt_cuss{1,index}{1,1};
    return_sat = opt_cuss{1,index}{2,1};
elseif al_i<7
    for i = 1:8
        filePath = ['D:\OneDrive\Yu\OneDrive\Experiment\Test\2EVRPLDTW-6\Data\',algorithm_name{al_i},'\',algorithm_name{al_i},'_',num_set{1},'_',num_depsat{2},'_100_TW_M2_D10_',num2str(i),'.mat'];
        disp(filePath);  % 输出生成的文件路径，检查是否正确
        load(filePath);
        Pop = result{1,2};
        Population = [Population,Pop];
    end
    Population = ArchiveUpdate(Population,100);
    Objs = Population.objs;
    % 轻度数值稳健：避免浮点微小差异导致“去不掉重”
    ObjsR = round(Objs, 10);
    [~, ia] = unique(ObjsR, 'rows', 'stable');
    Population = Population(ia);
    [~, index] = min(sum(Population.objs, 2));
    opt_sat = Population(index).add{1,1}{1,1};
    opt_cus = Population(index).add{1,1}{2,1};  %只有起始卫星和客户，每个路段最后没有返回卫星
    return_sat = Population(index).add{1,1}{3,1};
else
    xlsx_path = 'F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data_handle\Noninfer_DSDE_vs_Gurobi.xlsx';
    default_prob = 'Ca3_23_30_TW';   % 默认显示
    Tg = readtable(xlsx_path,'Sheet','Gurobi_ND_routes','VariableNamingRule','preserve');
    mask = string(Tg.Problem)==default_prob;
    sub = Tg(mask,:);
    [~,bestRow]=min(sub.f1+sub.f2);
    row=sub(bestRow,:);
    parseRoute=@(s)str2double(regexp(string(s),'\d+','match'));
    opt_sat=parseRoute(row.("First-echelon"));
    g_second=parseRoute(row.("Second-echelon"));
    [opt_cus,return_sat]=split_gurobi_second(g_second,num_sat,dis_sc);
end



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

% ------------------ 一级路径（修正版：多回路 + 补齐回仓库 + 正确映射） ------------------
if ~isempty(opt_sat)
    num_dep = size(coord_dep,1);
    nodes1 = opt_sat(:)';              % 原始编码：dep<=num_dep, sat>num_dep
    nodes1 = nodes1(nodes1>0);

    % 以“仓库节点(<=num_dep)”作为回路分隔/起点标记
    dep_pos = find(nodes1 <= num_dep);
    if isempty(dep_pos)
        warning('opt_sat does not contain any depot marker (<=num_dep).');
    else
        for rr = 1:numel(dep_pos)
            sidx = dep_pos(rr);
            eidx = numel(nodes1);
            if rr < numel(dep_pos)
                eidx = dep_pos(rr+1) - 1;
            end

            tour = nodes1(sidx:eidx);
            if isempty(tour), continue; end

            % tour(1) 必须是出发仓库
            if tour(1) > num_dep
                % 极端容错：如果这一段没从仓库开始，则跳过
                continue;
            end

            % 补齐回到出发仓库（闭环）
            if tour(end) ~= tour(1)
                tour = [tour, tour(1)];
            end

            % 映射为坐标：仓库直接索引；卫星要 -num_dep
            xy1 = zeros(numel(tour),2);
            for k = 1:numel(tour)
                if tour(k) <= num_dep
                    xy1(k,:) = coord_dep(tour(k),:);
                else
                    sid = tour(k) - num_dep;  % 关键修正
                    xy1(k,:) = coord_sat(sid,:);
                end
            end

            plot(xy1(:,1), xy1(:,2), '--b', 'LineWidth', 1.6);
            hold on;
        end
    end
end


% ------------------ 二级路径 ------------------
plot_second_layer(vrp2e,opt_cus,return_sat,coord_sat,coord_cus,dis_sc,E,V,color,colors,Energy_scale);
hold off;
xlabel('x-coordinate','FontWeight','bold');
ylabel('y-coordinate','FontWeight','bold');
axis equal; grid on; box on;


%% =================== 二级绘图函数（修正版） ===================
function plot_second_layer(vrp2e,opt_cus,return_sat,coord_sat,coord_cus,dis_sc,E,V,color,colors,Energy_scale)
    num_sat = size(coord_sat,1);
    if isempty(opt_cus), return; end

    Cc = 9.81^3 / sqrt(2 * 1.225 * (pi * 10^2) * 4);

    route2 = opt_cus(:)'; 
    route2 = route2(route2>0);

    % 子回路起点：卫星标记 (<= num_sat)
    sat_pos = find(route2 <= num_sat);

    subId = 0;
    for rr = 1:numel(sat_pos)
        sidx = sat_pos(rr);
        eidx = numel(route2);
        if rr < numel(sat_pos)
            eidx = sat_pos(rr+1) - 1;
        end

        subId = subId + 1;
        sub_raw = route2(sidx:eidx);       % [startSat, cus..., cus...]（不含返回卫星）
        if numel(sub_raw) < 2, continue; end

        startSat = sub_raw(1);             % 1..num_sat
        cus_list = sub_raw(2:end);         % 都应 > num_sat（全局编码）
        if isempty(cus_list), continue; end

        % --- 返回卫星：优先用 return_sat(subId)，否则用最后客户最近卫星 ---
        if numel(return_sat) >= subId && return_sat(subId) > 0
            endSat = return_sat(subId);    % 1..num_sat
        else
            lastCus = cus_list(end);       % 全局编号 > num_sat
            [~, endSat] = min(dis_sc(lastCus, 1:num_sat));
        end

        % 子回路补齐返回 endSat（关键：不是回 startSat）
        sub_full = [startSat, cus_list, endSat];

        % ---- 坐标映射：卫星直接索引；客户要 -num_sat ----
        len_nodes = numel(sub_full);
        xy = zeros(len_nodes,2);
        for k = 1:len_nodes
            if sub_full(k) <= num_sat
                xy(k,:) = coord_sat(sub_full(k),:);
            else
                cid = sub_full(k) - num_sat;   % 关键修正
                xy(k,:) = coord_cus(cid,:);
            end
        end

        % ---- 分段距离（使用全局编号索引 dis_sc）----
        dist_leg = dis_sc(sub2ind(size(dis_sc), sub_full(1:end-1), sub_full(2:end)));

        % ---- 分段载重（保持你的原 energy 接口：用全局编号传入）----
        load_profile = energy(vrp2e, sub_full);

        % ---- 能耗（你原来这里 wait_h=0；若你后面要加悬停，把 wait_h 换成真实等待小时即可）----
        fly_h  = dist_leg / V;                 % 小时
        wait_h = zeros(1, len_nodes-1);        % 小时（目前为0，后续可替换为真实悬停/等待）

        P = (5 + load_profile).^(3/2) * Cc;
        Eseg = P .* (fly_h + wait_h);          % 分段能耗
        E_cum = [0, cumsum(Eseg)];
        EnergyRem = (E - E_cum) / E;
        EnergyRem(EnergyRem<0) = 0;

        % ---- 画渐变线段 ----
        for k = 1:(len_nodes-1)
            EA = EnergyRem(k); 
            EB = EnergyRem(k+1);

            [~, idxA] = min(abs(Energy_scale - EA));
            [~, idxB] = min(abs(Energy_scale - EB));

            num = max(2, abs(idxB-idxA)+1);
            idxLine = round(linspace(idxA, idxB, num));

            x = linspace(xy(k,1), xy(k+1,1), num);
            y = linspace(xy(k,2), xy(k+1,2), num);

            for p = 1:(num-1)
                plot(x(p:p+1), y(p:p+1), 'Color', colors(idxLine(p),:), 'LineWidth', 3);
                hold on;
            end

            ccid = min(size(color,1), max(1, ceil(load_profile(k)/2 + 0.01)));
            plot(xy(k:k+1,1), xy(k:k+1,2), '--', 'LineWidth', 1.0, 'Color', color(ccid,:));
        end
    end
end


%% =================== Split Gurobi Second ===================
function [cus_noReturn,return_sat]=split_gurobi_second(seq,num_sat,dis_sc)
    cus_noReturn=[]; return_sat=[];
    i=1;
    while i<=numel(seq)
        if seq(i)>num_sat, error('Parse error'); end
        s0=seq(i); i=i+1; customers=[];
        while i<=numel(seq)&&seq(i)>num_sat
            customers(end+1)=seq(i); i=i+1;
        end
        if i<=numel(seq)&&seq(i)<=num_sat
            s_end=seq(i); i=i+1;
        else
            lastCus=customers(end);
            [~,s_end]=min(dis_sc(lastCus,1:num_sat));
        end
        cus_noReturn=[cus_noReturn,s0,customers];
        return_sat=[return_sat,s_end];
    end
end

%% =================== ArchiveUpdate ===================
function Population=ArchiveUpdate(Population,N)
    fIndex=all(Population.cons<=0,2);
    Population=Population(fIndex);
    if isempty(Population),return;end
    Population=Population(NDSort(Population.objs,1)==1);
    Population=Population(randperm(length(Population)));
    PCObj=Population.objs; nND=length(Population);
    if length(Population)>N
        fmax=max(PCObj,[],1); fmin=min(PCObj,[],1);
        PCObj=(PCObj-repmat(fmin,nND,1))./repmat(fmax-fmin,nND,1);
        d=pdist2(PCObj,PCObj); d(logical(eye(length(d))))=inf;
        sd=sort(d,2); r=mean(sd(:,min(3,size(sd,2)))); R=min(d./r,1);
        while length(Population)>N
            [~,worst]=max(1-prod(R,2));
            Population(worst)=[]; R(worst,:)=[]; R(:,worst)=[];
        end
    end
end