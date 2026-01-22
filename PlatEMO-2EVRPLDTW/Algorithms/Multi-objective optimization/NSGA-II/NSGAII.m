classdef NSGAII < ALGORITHM
% <multi> <real/integer/label/binary/permutation> <constrained/none>
% Nondominated sorting genetic algorithm II

%------------------------------- Reference --------------------------------
% K. Deb, A. Pratap, S. Agarwal, and T. Meyarivan, A fast and elitist
% multiobjective genetic algorithm: NSGA-II, IEEE Transactions on
% Evolutionary Computation, 2002, 6(2): 182-197.
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
            Problem.data.algorithm  = class(Algorithm);
            Population = Problem.Initialization();
            [~,FrontNo,CrowdDis] = EnvironmentalSelection(Population,Problem.N);
            gen = [1,2,4,6,8,10,12,14,16,18,20];
            k = 1;
            I = [];
            S = HV( Population,Problem.optimum);
            I = [I,S];
            %% Optimization
            while Algorithm.NotTerminated(Population)
                if mod(Problem.FE,floor(Problem.maxFE/1000))==0
                    S = IGD( Population,Problem.optimum);
                    I = [I,S];
                end
                % if Problem.FE/Problem.N == gen(k) || Problem.FE > Problem.maxFE-2*Problem.N
                %     pop_gen = gen(k); % 获取类名
                %     pop = Population.objs;
                %     fileName1 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Convergence\\NSGAII_Ca2315_%d.mat', pop_gen);
                %     save(fileName1, 'pop', '-V7.3'); % 保存第一个文件
                %     k = k+1;
                % end 
                % if Problem.FE >= Problem.maxFE
                %     fileName2 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Convergence\\NSGAII_Ca2315_cov.mat');
                %     save(fileName2, 'I', '-V7.3'); % 保存第一个文件  
                % end
                MatingPool = TournamentSelection(2,Problem.N,FrontNo,-CrowdDis);
                Offspring  = OperatorGA(Problem,Population(MatingPool));
                [Population,FrontNo,CrowdDis] = EnvironmentalSelection([Population,Offspring],Problem.N);
                CV = CalCV(Population.cons);
                if Problem.FE >= Problem.maxFE-1
                    Pop = Population.objs();
                    Data = I;
                end
            end
        end
    end
end

function CV = CalCV(CV_Original)
    CV_Original = max(CV_Original,0);
    CV = CV_Original./max(CV_Original,[],1);
    CV(:,isnan(CV(1,:))) = 0;
    CV = mean(CV,2);
end