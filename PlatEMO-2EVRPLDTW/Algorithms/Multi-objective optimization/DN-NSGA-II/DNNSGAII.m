classdef DNNSGAII < ALGORITHM
% <multi> <real/integer> <multimodal>
% Decision space based niching NSGA-II

%------------------------------- Reference --------------------------------
% J. Liang, C. Yue, and B. Qu. Multimodal multi-objective optimization: A
% preliminary study, Proceedings of the IEEE Congress on Evolutionary
% Computation, 2016, 2454-2461.
%------------------------------- Copyright --------------------------------
% Copyright (c) 2024 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------

    methods
        function main(Algorithm,Problem)
            %% Generate random population
            Population = Problem.Initialization();
            [~,FrontNo,CrowdDis] = EnvironmentalSelection(Population,Problem.N);
            I1 = [];
            c = [];
            S1 = HV(Population,Problem.optimum);
            I1 = [I1,S1];
            c = [c, sum(sum(Population.cons))];
            %% Optimization
            while Algorithm.NotTerminated(Population)
                if mod(Problem.FE,Problem.N)==0
                    S = HV(Population,Problem.optimum);
                    I1 = [I1,S];
                    c = [c, sum(sum(Population.cons))];
                end
                if Problem.FE >= Problem.maxFE-Problem.N
                    problem_name = class(Problem); % 获取类名
                    fileName1 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\DNNSGAII_%s_HV.mat', problem_name);
                    save(fileName1, 'I1', '-V7.3'); % 保存第一个文件
                    fileName2 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\DNNSGAII_%s_CV.mat', problem_name);
                    save(fileName2, 'c', '-V7.3'); % 保存第一个文件
                end
                MatingPool = TournamentSelection_Mod(round(Problem.N/2),round(Problem.N/2),Population.decs,FrontNo,-CrowdDis); %收敛最佳的决策空间拥挤解
                Offspring  = OperatorGA(Problem,Population(MatingPool));
                [Population,FrontNo,CrowdDis] = EnvironmentalSelection([Population,Offspring],Problem.N); %非支配层决策空间稀疏解、目标空间稀疏解
            end
        end
    end
end