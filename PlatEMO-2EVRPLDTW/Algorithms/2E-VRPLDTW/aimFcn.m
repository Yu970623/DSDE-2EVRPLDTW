% 1无人机续航约束，2无人机载重约束，3整体约束
function [obj1,obj2,c1,c2,c3,c4,c5,All_routes,split_point,operator_num]=aimFcn(x,option,data,init,operator_num)
    list_served = [];
    list_hold = 1:data.num_cus;
    need_newroute2 = true;
    All_routes2 = [];
    Inter_routes = [];
    Exter_routes = [];
    dis_sat_cus = data.dis_second(1:data.num_sat,1:data.num_cus);
    dis_cuss = data.dis_second(data.num_sat+1:end,1:data.num_cus);
    num_drones = 0;
    non_cus = 0;
    split_point = zeros(1,length(x));
    %seg_route_RB = zeros(1,length(x));
    k = 1;
    RBB = 1;
    if init || isempty(operator_num)
        operator_num = randi([0, 1], 1, 14);  % 均匀随机生成 0 或 1作为操作符
        l = 1;
    end
    
    while ~isempty(list_hold)      %仍有待服务客户
        if need_newroute2    %找到距卫星最近的待插入客户
            new_cus = list_hold(find(x(list_hold) == max(x(list_hold)), 1));
            near_sat = find(dis_sat_cus(:,new_cus)==max(dis_sat_cus(:,new_cus)),1);
            list_served = [list_served,new_cus];       %更新服务过客户列表
            list_hold(list_hold == new_cus) = [];      %删除待服务客户列表
            temp_route = [near_sat, new_cus+data.num_sat];    %临时子路段
            need_newroute2 = false;      %暂不需要新路段
            num_drones = num_drones+1;
            distance_next = dis_cuss(new_cus,:);
            distance_next = distance_next / max(distance_next);
            distance_next(distance_next==0)=Inf;
            x = x+(1-distance_next);
            x = x / max(max(x),0);
        elseif ~isempty(list_hold)    %将其他待插入客户插入当前未确定路段中
            temp = setdiff(1:numel(x), list_served);     %去除已服务客户连接关系
            [~, idx] = sort(x(temp), 'descend');         %按照选择概率从高到低排序
            Priori_cus = temp(idx);
            mincost = ones(1,min(20,length(Priori_cus)));
            temp_route1 = cell(1,min(20,length(Priori_cus)));
            RB = ones(1,min(20,length(Priori_cus)));
            for i = 1:min(20,length(Priori_cus))       %按照优先级逐一插入客户并确定最佳位置
                [temp_route1{i},mincost(i),RB(i),~] = instert_cus(data,data.vrp2e,temp_route,Priori_cus(i)+data.num_sat);  %客户插入函数，返回最佳位置
            end
            if init  %初始化路段
                if all(isinf(mincost)) || rand() < 1 - RBB
                    need_newroute2 = true;
                    All_routes2 = [All_routes2, temp_route];
                    split_point(non_cus+length(temp_route)-1)=1;
                    non_cus = length(temp_route)-1;
                    seg_route_RB(k) = RBB;
                    k = k+1;
                else
                    [~, j] = min(mincost);
                    temp_route = temp_route1{j};
                    RBB = RB(j);
                    list_served = [list_served,Priori_cus(j)];
                    list_hold(list_hold == Priori_cus(j)) = [];
                    distance_next = dis_cuss(Priori_cus(j),:);
                    distance_next = distance_next / max(distance_next);
                    distance_next(distance_next==0)=Inf;
                    x = x+(1-distance_next);
                    x = x / max(max(x),0);
                end
            else
                if all(isinf(mincost)) || split_point(non_cus+length(temp_route)-1)==1 %复现路段
                    need_newroute2 = true;
                    All_routes2 = [All_routes2, temp_route];
                    non_cus = length(temp_route)-1;
                else
                    [~, j] = min(mincost);
                    temp_route = temp_route1{j};
                    list_served = [list_served,Priori_cus(j)];
                    list_hold(list_hold == Priori_cus(j)) = [];
                    distance_next = dis_cuss(Priori_cus(j),:);
                    distance_next = distance_next / max(distance_next);
                    distance_next(distance_next==0)=Inf;
                    x = x+(1-distance_next);
                    x = x / max(max(x),0);
                end
            end
        end
        if isempty(list_hold)
            All_routes2 = [All_routes2, temp_route]; 
            seg_route_RB(k) = RBB;
        end
    end
    [equip2,energy2,c_w,c_e] = msecond(data.vrp2e,All_routes2);
    %                                   类  内  外  内路 内客 外路 外客
    % A = inter_opera(data,All_routes2,[0,0,1,1,0,0,1,0,1,1,1,0,1,1]);
    %                                   1 2 3 4 5 6 7 8 9 0 1 2 3 4
    while l < 30
        binary_str = strrep(num2str(operator_num(1:2)), ' ', '');
        switch base2dec(binary_str, 2) %操作类型
            case 0  %仅路段内部操作
                Inter_routes = inter_opera(data,All_routes2,operator_num);
                [equip_inter,energy_inter,w_inter,c_inter] = msecond(data.vrp2e,Inter_routes);
                if (equip_inter+energy_inter)<(equip2+energy2) && (w_inter+c_inter)<=(c_w+c_e)
                    All_routes2 = Inter_routes;
                    l = 0;
                else
                    operator_num = randi([0, 1], 1, 14);
                    l = l+1;
                end
            case 1  %仅路段间操作
                Exter_routes = exter_opera(data,All_routes2,operator_num,seg_route_RB);
    
                [equip_exter,energy_exter,w_exter,c_exter] = msecond(data.vrp2e,Exter_routes);
                if (equip_exter+energy_exter)<(equip2+energy2) && (w_exter+c_exter)<=(c_w+c_e)
                    All_routes2 = Exter_routes;
                    l = 0;
                else
                    operator_num = randi([0, 1], 1, 14);
                    l = l+1;
                end
            case 2  %先外后内
                Exter_routes = exter_opera(data,All_routes2,operator_num,seg_route_RB);
                Inter_routes = inter_opera(data,Exter_routes,operator_num);
    
                [equip_inter,energy_inter,w_inter,c_inter] = msecond(data.vrp2e,Inter_routes);
                if (equip_inter+energy_inter)<(equip2+energy2) && (w_inter+c_inter)<=(c_w+c_e)
                    All_routes2 = Inter_routes;
                    l = 0;
                else
                    operator_num = randi([0, 1], 1, 14);
                    l = l+1;
                end
            case 3  %内外同时择其优
                Inter_routes = inter_opera(data,All_routes2,operator_num);
                Exter_routes = exter_opera(data,All_routes2,operator_num,seg_route_RB);
    
                [equip_inter,energy_inter,w_inter,c_inter] = msecond(data.vrp2e,Inter_routes);
                [equip_exter,energy_exter,w_exter,c_exter] = msecond(data.vrp2e,Exter_routes);
                % 当前总成本和约束
                current_cost = equip2 + energy2;
                current_constraint = c_w + c_e;
                
                % 计算各方案的总成本和约束
                costs = [equip_inter + energy_inter, equip_exter + energy_exter];
                constraints = [w_inter + c_inter, w_exter + c_exter];
                routes = {Inter_routes, Exter_routes};  % 候选解
                
                % 找出所有满足条件的候选解（成本更低且约束更小）
                valid_indices = find((costs < current_cost) & (constraints < current_constraint));
                
                if ~isempty(valid_indices)
                    % 从候选解中选择成本最低的一个
                    [~, best_sub_idx] = min(costs(valid_indices));
                    best_route = routes{valid_indices(best_sub_idx)};
                    All_routes2 = best_route;
                    l = 0;
                else
                    operator_num = randi([0, 1], 1, 14);
                    l = l+1;
                end
        end
    end

    [equip_cost2,energy_cost2,c_weight,c_energy] = msecond(data.vrp2e,All_routes2);
    [seq_sat,cap_sat] = insert_sat(data.vrp2e,All_routes2);
    All_routes = {seq_sat;All_routes2};
    [equip_cost1,energy_cost1,fvlt1] = mfirst(data.vrp2e,seq_sat,cap_sat);
    obj1 = equip_cost1+equip_cost2;
    obj2 = energy_cost1+energy_cost2;
    c1=c_weight;
    c2=c_energy;
    c3=fvlt1;
    c4=time_violate(seq_sat,All_routes2,data,data.vrp2e);
    c5 = c1 + c2 + c3 + c4;
end


function [All_routes2]= inter_opera(data,All_routes2,operator_num)
    loc_sats = find(All_routes2 <= data.num_sat);
    num_segroute = length(loc_sats);  %所有子路段条数
    binary_str = strrep(num2str(operator_num(7:8)), ' ', '');
    inter_opera_routes = find(mod(1:num_segroute, round(num_segroute/5)+base2dec(binary_str, 2)) == 0); %执行操作的路段序号
    split_points = [loc_sats, length(All_routes2)+1];
    segroutes = arrayfun(@(i) All_routes2(split_points(i):split_points(i+1)-1), 1:length(split_points)-1, 'UniformOutput', false); %分割子路段
    binary_str = strrep(num2str(operator_num(3:4)), ' ', '');
    switch base2dec(binary_str, 2)     %内部操作类型
        case 0  %子路段翻转
            for i = inter_opera_routes
                route = segroutes{i}(2:end);
                binary_str = strrep(num2str(operator_num(9:10)), ' ', '');
                inter_opera_cus = find(mod(1:length(route), ceil((base2dec(binary_str, 2)+1)/2)) == 0);
                if numel(inter_opera_cus) > 1
                    for j = 2:2:length(inter_opera_cus)-1
                        route(inter_opera_cus(j):inter_opera_cus(j+1)) = route(inter_opera_cus(j+1):-1:inter_opera_cus(j));
                    end
                end
                segroutes{i} = [segroutes{i}(1),route];
            end
        case 1  %节点位置交换
            for i = inter_opera_routes
                route = segroutes{i}(2:end);
                binary_str = strrep(num2str(operator_num(9:10)), ' ', '');
                inter_opera_cus = find(mod(1:length(route), ceil((base2dec(binary_str, 2)+1)/2)) == 0);
                inter_opera_cus = inter_opera_cus(1:2*floor(length(inter_opera_cus)/2));
                for j = 1:2:length(inter_opera_cus)
                    route([inter_opera_cus(j), inter_opera_cus(j+1)]) = route([inter_opera_cus(j+1), inter_opera_cus(j)]);
                end
                segroutes{i} = [segroutes{i}(1),route];
            end
        case 2  %子路段平移
            for i = inter_opera_routes
                route = segroutes{i}(2:end);
                binary_str = strrep(num2str(operator_num(9:10)), ' ', '');
                inter_opera_cus = find(mod(1:length(route), ceil((base2dec(binary_str, 2)+1)/2)) == 0);
                inter_opera_cus = inter_opera_cus(1:2*floor(length(inter_opera_cus)/2));

                % 处理每对片段
                for j = 1:2:length(inter_opera_cus)-1
                    if j+2 <= length(inter_opera_cus)  % 有下一对可交换
                        % 交换相邻片段内容
                        seg1 = inter_opera_cus(j):inter_opera_cus(j+1);
                        seg2 = inter_opera_cus(j+2):inter_opera_cus(j+3);
                        route([seg1,seg2]) = route([seg2,seg1]);
                    else
                        % 翻转最后单个片段
                        seg = inter_opera_cus(j):inter_opera_cus(j+1);
                        route(seg) = fliplr(route(seg));
                    end
                end
                segroutes{i} = [segroutes{i}(1),route];
            end
        case 3  %随机扰动（移除后重新插入当前路段）
            for i = inter_opera_routes
                route = segroutes{i}(2:end);
                binary_str = strrep(num2str(operator_num(9:10)), ' ', '');
                inter_opera_cus = find(mod(1:length(route), ceil((base2dec(binary_str, 2)+1)/2)) == 0);
                wait_cus = fliplr(route(sort(inter_opera_cus)));  % 先排序后翻转
                route(inter_opera_cus) = [];  % 删除原位置元素
                route = [segroutes{i}(1),route];
                for j = wait_cus     %按照优先级逐一插入客户并确定最佳位置
                    [route,cost,~,~] = instert_cus(data,data.vrp2e,route,j);  %客户插入函数，返回最佳位置
                    if isinf(cost)
                        route = [route,j];
                    end
                end
                segroutes{i} = route;
            end
    end
end

function [All_routes2]= exter_opera(data,All_routes2,operator_num,seg_route_RB)
    loc_sats = find(All_routes2 <= data.num_sat);
    num_segroute = length(loc_sats);  %所有子路段条数
    binary_str = strrep(num2str(operator_num(11:12)), ' ', '');
    exter_opera_routes = find(mod(1:num_segroute, round(num_segroute/5)+base2dec(binary_str, 2)) == 0); %执行操作的路段序号
    split_points = [loc_sats, length(All_routes2)+1];
    segroutes = arrayfun(@(i) All_routes2(split_points(i):split_points(i+1)-1), 1:length(split_points)-1, 'UniformOutput', false); %分割子路段
    seg_route_RB(find(seg_route_RB ~= 0, 1, 'last')+1:end) = [];
    binary_str = strrep(num2str(operator_num(5:6)), ' ', '');
    switch base2dec(binary_str, 2)    %外部操作类型
        case 0  %单一客户迁移
            if ~isempty(exter_opera_routes)
                for i = length(exter_opera_routes)
                    route = segroutes{exter_opera_routes(i)}(2:end);
                    binary_str = strrep(num2str(operator_num(13:14)), ' ', '');
                    exter_opera_cus{i} = find(mod(1:length(route), ceil((base2dec(binary_str, 2)+1)/2)) == 0);
                end
            end
            % 确保偶数个操作路段
            exter_opera_routes = exter_opera_routes(1:2*floor(length(exter_opera_routes)/2));
            
            % 相邻路段客户迁移
            for i = 1:2:length(exter_opera_routes)-1
                % 获取当前路段和下一路段
                route1 = segroutes{exter_opera_routes(i)}(2:end);
                route2 = segroutes{exter_opera_routes(i+1)}(2:end);
                
                % 移除并翻转客户
                wait_cus1 = fliplr(route1(sort(exter_opera_cus{i})));
                wait_cus2 = fliplr(route2(sort(exter_opera_cus{i+1})));
                
                % 从原路段移除客户
                route1(exter_opera_cus{i}) = [];
                route2(exter_opera_cus{i+1}) = [];
                route1 = [segroutes{exter_opera_routes(i)}(1),route1];
                route2 = [segroutes{exter_opera_routes(i+1)}(1),route2];
                % 将wait_cus1插入route2
                for j = wait_cus1
                    [route2, cost, ~, ~] = instert_cus(data, data.vrp2e, route2, j);
                    if isinf(cost)
                        route2 = [route2, j];
                    end
                end
                
                % 将wait_cus2插入route1
                for j = wait_cus2
                    [route1, cost, ~, ~] = instert_cus(data, data.vrp2e, route1, j);
                    if isinf(cost)
                        route1 = [route1, j];
                    end
                end
                % 更新路段
                segroutes{exter_opera_routes(i)} = route1;
                segroutes{exter_opera_routes(i+1)} = route2;
            end
        case 1  %单一客户交换
            % 初始化操作路段和客户索引
            for i = 1:length(exter_opera_routes)
                route = segroutes{exter_opera_routes(i)};
                binary_str = strrep(num2str(operator_num(13:14)), ' ', '');
                exter_opera_cus{i} = find(mod(1:length(route), round(base2dec(binary_str, 2)/2)) == 0);
            end
            
            % 确保偶数个操作路段
            exter_opera_routes = exter_opera_routes(1:2*floor(length(exter_opera_routes)/2));
            
            % 相邻路段客户交换
            for i = 1:2:length(exter_opera_routes)-1
                % 获取当前路段和下一路段
                route1 = segroutes{exter_opera_routes(i)};
                route2 = segroutes{exter_opera_routes(i+1)};
                
                % 确定最小交换数量
                min_swap_num = min(length(exter_opera_cus{i}), length(exter_opera_cus{i+1}));
                
                % 获取要交换的客户索引（取前min_swap_num个）
                swap_idx1 = exter_opera_cus{i}(1:min_swap_num);
                swap_idx2 = exter_opera_cus{i+1}(1:min_swap_num);
                
                % 执行交换
                temp_cus = route1(swap_idx1);
                route1(swap_idx1) = route2(swap_idx2);
                route2(swap_idx2) = temp_cus;
                
                % 更新路段
                segroutes{exter_opera_routes(i)} = route1;
                segroutes{exter_opera_routes(i+1)} = route2;
            end
        case 2  %子路段迁移
            % 初始化操作路段和客户索引
            for i = 1:length(exter_opera_routes)
                route = segroutes{exter_opera_routes(i)};
                binary_str = strrep(num2str(operator_num(13:14)), ' ', '');
                exter_opera_cus{i} = find(mod(1:length(route), round(base2dec(binary_str, 2)/2)) == 0);
                
                % 确保每个exter_opera_cus{i}有偶数个元素（成对构成子路段）
                exter_opera_cus{i} = exter_opera_cus{i}(1:2*floor(length(exter_opera_cus{i})/2));
            end
            
            % 确保偶数个操作路段
            exter_opera_routes = exter_opera_routes(1:2*floor(length(exter_opera_routes)/2));
            
            % 相邻路段子路段交换
            for i = 1:2:length(exter_opera_routes)-1
                % 获取当前路段和下一路段
                route1 = segroutes{exter_opera_routes(i)};
                route2 = segroutes{exter_opera_routes(i+1)};
                
                % 确定最小子路段对数
                min_seg_pairs = min(length(exter_opera_cus{i})/2, length(exter_opera_cus{i+1})/2);
                
                % 执行子路段交换
                for j = 1:min_seg_pairs
                    % 获取要交换的子路段索引
                    seg1_start = exter_opera_cus{i}(2*j-1);
                    seg1_end = exter_opera_cus{i}(2*j);
                    seg2_start = exter_opera_cus{i+1}(2*j-1);
                    seg2_end = exter_opera_cus{i+1}(2*j);
                    
                    % 提取子路段
                    seg1 = route1(seg1_start:seg1_end);
                    seg2 = route2(seg2_start:seg2_end);
                    
                    % 执行交换
                    route1(seg1_start:seg1_end) = seg2;
                    route2(seg2_start:seg2_end) = seg1;
                end
                
                % 更新路段
                segroutes{exter_opera_routes(i)} = route1;
                segroutes{exter_opera_routes(i+1)} = route2;
            end
        case 3  %子路段合并
            % 找出剩余续航比率>0.5的路段并排序
            high_RB_idx = find(seg_route_RB > 0.5);
            [~, sorted_idx] = sort(seg_route_RB(high_RB_idx), 'descend');
            high_RB_idx = high_RB_idx(sorted_idx);
            
            % 确保偶数个待合并路段
            high_RB_idx = high_RB_idx(1:2*floor(length(high_RB_idx)/2));
            
            % 执行路段合并
            for i = 1:2:length(high_RB_idx)-1
                % 获取待消除路段和接收路段
                route1 = segroutes{high_RB_idx(i)};
                route2 = segroutes{high_RB_idx(i+1)};
                
                route3 = route1(1);
                % 转移客户（除第一个客户外）
                wait_cus = fliplr(route1(2:end));
                for j = wait_cus
                    [route2, cost, ~, ~] = instert_cus(data, data.vrp2e, route2, j);
                    if isinf(cost)
                        route3 = [route3, j];
                    end
                end
                
                % 更新路段
                segroutes{high_RB_idx(i+1)} = route2;
                if isscalar(route3)
                    segroutes{high_RB_idx(i)} = [];
                else
                    segroutes{high_RB_idx(i)} = route3;
                end
                  % 标记为空，后续统一删除
            end
            % 移除空路段
            segroutes = segroutes(~cellfun('isempty', segroutes));
    end
end