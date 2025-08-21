% 1无人机续航约束，2无人机载重约束，3整体约束
function [obj1,obj2,c1,c2,c3,c4,c5,All_routes]=aimFcn0(x,option,data)
    list_served = [];
    list_hold = 1:data.num_cus;
    need_newroute2 = true;
    All_routes2 = [];
    num_drones = 0;
    while ~isempty(list_hold)      %仍有待服务客户
        if need_newroute2    %找到距卫星最近的待插入客户
            dis_sat_cus = data.dis_second(1:data.num_sat,1:data.num_cus);
            new_cus = list_hold(find(x(list_hold) == max(x(list_hold)), 1));
            near_sat = find(dis_sat_cus(:,new_cus)==max(dis_sat_cus(:,new_cus)),1);
            list_served = [list_served,new_cus];       %更新服务过客户列表
            list_hold(list_hold == new_cus) = [];      %删除待服务客户列表
            temp_route = [near_sat, new_cus+data.num_sat];    %临时子路段
            need_newroute2 = false;      %暂不需要新路段
            num_drones = num_drones+1;
        elseif ~isempty(list_hold)    %将其他待插入客户插入当前未确定路段中
            temp = setdiff(1:numel(x), list_served);     %去除已服务客户连接关系
            [~, idx] = sort(x(temp), 'descend');         %按照选择概率从高到低排序
            Priori_cus = temp(idx);                             %排序客户序列
            for temp_cus = Priori_cus(1:min(20,length(Priori_cus)))       %按照优先级逐一插入客户并确定最佳位置
                [temp_route1,~,~,need_newroute2] = instert_cus(data,data.vrp2e,temp_route,temp_cus+data.num_sat);  %客户插入函数，返回最佳位置
                if isequal(temp_route1, temp_route) && need_newroute2    %无法插入当前客户temp_cus
                    if isequal(temp_cus, Priori_cus(min(20,length(Priori_cus))))       %最后一个客户也无法插入当前路段
                        All_routes2 = [All_routes2, temp_route];  %保留当前已有路段
                        break;
                    else
                        continue;
                    end
                else
                    temp_route = temp_route1;       %成功插入当前客户
                    list_served = [list_served,temp_cus];
                    list_hold(list_hold == temp_cus) = [];
                    break;
                end
            end
        end
        if isempty(list_hold)
            All_routes2 = [All_routes2, temp_route]; 
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
