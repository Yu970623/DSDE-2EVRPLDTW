function Fitness = CalFitness(PopObj,PopCon,processcon)
% Calculate the fitness of each solution

%------------------------------- Copyright --------------------------------
% Copyright (c) 2022 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------

    N = size(PopObj,1);
    if nargin == 1
        CV = zeros(N,1);
    elseif nargin == 2
        CV = sum(max(0,PopCon),2);    % CV：所有约束的和
    else
        PopCon = max(0,PopCon(:,processcon));
        CV = sum(PopCon,2);
    end
%    CV(CV<=epsilon) = 0;

    %% Detect the dominance relation between each two solutions 计算每两个个体之间的支配关系
    Dominate = false(N);      % Dominate：N乘N的矩阵，保存每对个体之间的支配关系
    for i = 1 : N-1
        for j = i+1 : N
            if CV(i) < CV(j)
                Dominate(i,j) = true;
            elseif CV(i) > CV(j)
                Dominate(j,i) = true;
            else
                k = any(PopObj(i,:)<PopObj(j,:)) - any(PopObj(i,:)>PopObj(j,:));
                if k == 1
                    Dominate(i,j) = true;
                elseif k == -1
                    Dominate(j,i) = true;
                end
            end
        end
    end
    
    %% Calculate S(i)
    S = sum(Dominate,2);    % S(i):强度值，等于受该个体所支配的解的数量
    
    %% Calculate R(i)  
    R = zeros(1,N);
    for i = 1 : N
        R(i) = sum(S(Dominate(:,i)));   % R(i):原始适应度值，等于支配该个体的所有个体的强度值之和，非支配解的R(i)为0
    end
    
    %% Calculate D(i)
    Distance = pdist2(PopObj,PopObj);
    Distance(logical(eye(length(Distance)))) = inf;   
    Distance = sort(Distance,2);
    D = 1./(Distance(:,floor(sqrt(N)))+2);    % D(i):密度值
    
    %% Calculate the fitnesses
    Fitness = R + D';     % 最终个体适应度值为原始适应度值和密度值之和
end