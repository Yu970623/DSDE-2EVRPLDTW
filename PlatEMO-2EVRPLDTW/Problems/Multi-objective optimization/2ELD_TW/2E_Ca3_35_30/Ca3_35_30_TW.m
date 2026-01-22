classdef Ca3_35_30_TW < PROBLEM
    % <multi/many> <real> <large/none> <constrained> <expensive/none>
    
    %------------------------------- Reference --------------------------------
    % Multiobjective Routing Optimization for Multimodal Transportation with Mixed Time Windows under Fuzzy Demand
	
	
    %------------------------------- Copyright --------------------------------
    % Copyright (c) 2024 BIMK Group. You are free to use the PlatEMO for
    % research purposes. All publications which use this platform or any code
    % in the platform should acknowledge the use of "PlatEMO" and reference "Ye
    % Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
    % for evolutionary multi-objective optimization [educational forum], IEEE
    % Computational Intelligence Magazine, 2017, 12(4): 73-87".
    %--------------------------------------------------------------------------
    properties(Access = public)
        data;
        option;
    end
    
    methods
        %% Default settings of the problem
        function Setting(obj)
            if isempty(obj.M); obj.M = 2; end
            if isempty(obj.D); obj.D = 10; end 
            obj.data.algorithm = [];
            obj.lower    = zeros(1,obj.D);
            obj.upper    = ones(1,obj.D);
            obj.encoding = ones(1,obj.D);

            
            %% 数据定义和读取
            set = 1;  %Ca数据集
            seq = 30;   %第几个2,3,15
            [vrp2e,timewindows,timeservice,demand,type,dis_ds,dis_sc,num_dep,num_sat,num_cus] = extractdata(set,seq);
            obj.data.vrp2e = vrp2e;
            obj.data.numN=num_dep+num_sat+num_cus;  %节点数量
            obj.data.num_dep = num_dep;             %仓库数量
            obj.data.num_sat = num_sat;             %卫星数量
            obj.data.num_cus = num_cus;             %客户数量
            obj.data.demand=demand;                 %客户需求量
            obj.data.type=type;                     %客户需求类型
            obj.data.Windows=timewindows;           %这个可能需要松弛
            obj.data.Services=timeservice;          %客户服务时间
            obj.data.dis_first = dis_ds(:,num_dep+1:end);   %一级路径矩阵
            obj.data.dis_second = dis_sc(:,num_sat+1:end);  %二级路径矩阵
            obj.data.v = [30,10];                   %一、二级路径的车辆速度
            obj.data.T1 = 60*obj.data.dis_first/obj.data.v(1); %一级路径的路段运行时长
            obj.data.T2 = 60*obj.data.dis_second/obj.data.v(2);%二级路径的路段运行时长
            obj.data.CW=[30,50];                    %时间窗惩罚系数

            obj.data.elite = 0.5*ones(1,obj.D);
            if obj.FE == 0
                obj.data.start_time = obj.data.Windows(:,1);
                obj.data.end_time = obj.data.Windows(:,2);
            end
            obj.data.maxE=550;  %最大电池容量
            obj.data.maxW=5;    %最大载重重量

            lb=0;
            ub=1;
            dim=obj.D;
            obj.option.lb=lb;
            obj.option.ub=ub;
            obj.option.dim=dim;
            if length(obj.option.lb)==1
                obj.option.lb=ones(1,obj.option.dim)*obj.option.lb;
                obj.option.ub=ones(1,obj.option.dim)*obj.option.ub;
            end
            obj.option.showIter=0;
        end
        
        %% Calculate objective values and constraint violations
        function Population = Evaluation(obj,varargin)

            PopDec = varargin{1};

            % 非线性动态参数
            obj.data.ratio = (1 + exp(-15 * ((obj.FE / (0.8*obj.maxFE)) - 0.5)));
            y = 0.5 - (0.5 ./ obj.data.ratio);

            obj.data.beta = 1 - y;  % Sigmoid增长
            obj.data.eta = 1 - (1 ./ obj.data.ratio);                       % 指数衰减

            if obj.FE > 0.8*obj.maxFE
                y = 0; 
            end
            
                        %% 精英解引导
            if isequal(obj.data.algorithm,'TEST') || isequal(obj.data.algorithm,'DSDE')
                %% 时间窗的动态放松（硬时间窗动态软化）
                obj.data.Windows(:,1)=obj.data.start_time.*(1-y);      %对时间窗进行更新：软时间窗下限 调低
                obj.data.Windows(:,2)=obj.data.end_time.*(1+y);      %对时间窗进行更新：软时间窗上限 调高
                % 随机选择行
                n_method1 = round(length(PopDec(:,1)) * obj.data.eta);
                idx_rand = randperm(length(PopDec(:,1)));
                obj.data.idx_method1 = idx_rand(1:n_method1);
                obj.data.idx_method2 = idx_rand(n_method1+1:end);
                % 方法1: 随机趋势调整
                if ~isempty(obj.data.idx_method1)
                    r = rand(length(obj.data.idx_method1), obj.D) - 0.5;
                    PopDec(obj.data.idx_method1, :) = PopDec(obj.data.idx_method1, :) + obj.data.eta * r .* (obj.data.elite - 0.5);
                end
                % 方法2: 自适应靠拢
                if ~isempty(obj.data.idx_method2)
                    diff = obj.data.elite - PopDec(obj.data.idx_method2, :);
                    PopDec(obj.data.idx_method2, :) = PopDec(obj.data.idx_method2, :) + obj.data.beta * abs(diff) .* diff;
                end
                
                % 向量化行级归一化
                min_vals = min(PopDec, [], 2);
                max_vals = max(PopDec, [], 2);
                ranges = max_vals - min_vals;
                ranges(ranges == 0) = 1;  % 处理全相等行
                PopDec = (PopDec - min_vals) ./ ranges;
                PopDec(isnan(PopDec)) = 0;
            end

            N = size(PopDec,1);     %种群个数   
            PopObj=zeros(N,2);  %目标函数个数2
            PopCon=zeros(N,5);  %约束条件个数5   无人机载重、无人机负载、卡车车辆数、时间窗违反、总约束违反
            route = cell(N,1); 
            if isequal(obj.data.algorithm,'TEST') || isequal(obj.data.algorithm,'DSDE')
                for i=1:N
                    [PopObj(i,1),PopObj(i,2),PopCon(i,1),PopCon(i,2),PopCon(i,3),PopCon(i,4),PopCon(i,5),route{i}] = aimFcn6(PopDec(i,:),obj.data);
                end
            else
                for i=1:N
                    [PopObj(i,1),PopObj(i,2),PopCon(i,1),PopCon(i,2),PopCon(i,3),PopCon(i,4),PopCon(i,5),route{i}] = aimFcn10(PopDec(i,:),obj.data);
                end
            end
            [~, idx] = min(sum(PopObj, 2));
            
            Population = SOLUTION(PopDec,PopObj,PopCon,route);
            obj.data.elite = PopDec(idx,:);
            obj.FE     = obj.FE + length(Population);
        end
        
        %% Generate points on the Pareto front
        function R = GetOptimum(obj,N)
            R=[50 6];
        end
    end
end