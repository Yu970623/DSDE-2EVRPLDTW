classdef DSDE < ALGORITHM
% <multi/many> <real/integer/label/binary/permutation> <constrained/none>
% Nondominated sorting genetic algorithm III

%------------------------------- Reference --------------------------------
% K. Deb and H. Jain, An evolutionary many-objective optimization algorithm
% using reference-point based non-dominated sorting approach, part I:
% Solving problems with box constraints, IEEE Transactions on Evolutionary
% Computation, 2014, 18(4): 577-601.
%------------------------------- Copyright --------------------------------
% Copyright (c) 2023 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------

    methods
        function main(Algorithm,Problem)
            %% Generate the reference points and random population
            Problem.data.algorithm  = class(Algorithm);
            [P_pre,ROC_P] =  Algorithm.ParameterSet(1,1);
            isCov = false;
            Population = Problem.Initialization();
            Archive = ArchiveUpdate(Population,Problem.N);
            maxgen = Problem.maxFE/Problem.N;
            mu = maxgen/2;
            sigma = 0;
            [W,Problem.N] = UniformPoint(Problem.N,Problem.M);
            Zmin    = min(Population.objs,[],1);
            I1 = [];
            S1 = HV(Population,Problem.optimum);
            I1 = [I1,S1];
            %% Optimization
            while Algorithm.NotTerminated(Population)
                if mod(Problem.FE,Problem.N)==0
                    S = HV(Population,Problem.optimum);
                    I1 = [I1,S];
                end
                if Problem.FE >= Problem.maxFE-2*Problem.N
                    problem_name = class(Problem); % 获取类名
                    fileName1 = sprintf('F:\\Onedrive\\Experiment\\2EVRPLDTW\\Cov\\DSDN_%s_HV.mat', problem_name);
                    save(fileName1, 'I1', '-V7.3'); % 保存第一个文件
                end
                if Problem.FE >= Problem.maxFE-Problem.N
                    problem_name = class(Problem); % 获取类名
                    fileName1 = sprintf('F:\\Onedrive\\Experiment\\2EVRPLDTW\\Cov\\DSDN_%s_HV.mat', problem_name);
                    save(fileName1, 'I1', '-V7.3'); % 保存第一个文件
                end
                MatingPool = TournamentSelection(2,Problem.N,sum(max(0,Population.cons),2));
                [Mat1,Mat2] = Neighbor_Sort(Population(MatingPool),Zmin);
                Offspring = OperatorDE(Problem,Population(MatingPool),Mat1,Mat2,{rand(),rand(),rand(),rand()});
                Zmin       = min([Zmin;Offspring.objs],[],1);
                R = (Problem.D^(1/3)) * (1 - exp(-((Problem.FE/Problem.N-mu)^2) / (2 * sigma^2)));
                epsilon = max(max(Population.cons))*Problem.data.eta*0.5;
                Population = Update_time(Population,Problem.data,epsilon);
                Archive = Update_time(Archive,Problem.data,epsilon);
                [Population,~] = EnvironmentalSelection([Population,Offspring],Problem.N,W,Zmin,R);
                Archive = ArchiveUpdate([Archive,Population,Offspring],Problem.N);
                mean_P = sum(sum(Population.objs,1))/Problem.N;
                if mod(Problem.FE/Problem.N,10)==0
                    [ROC_P,P_pre] = Detect(P_pre,mean_P);
                end
                PND = Cal_PND(Population); 
                if (PND > 0.99 && ROC_P<1e-3) && ~isCov
                    sigma = floor(abs(mu-Problem.FE/Problem.N)/3);
                    isCov = true;
                end
            end
        end
    end
end

function [ROC,pre] = Detect(pre,now)
    ROC = abs((now-pre)/pre);
    pre = now; 
end
function PND =  Cal_PND(Population)
    [FrontNo,~]=NDSort(Population.objs,inf);  
    ND=size(find(FrontNo==1),2);    
    PND=ND/length(Population)  ;       
end
function [Mat1,Mat2] = Neighbor_Sort(Population,Zmin)
    Objs = (Population.objs - Zmin) ./ sqrt(sum((Population.objs - Zmin).^2, 2));
    CosV = (Objs * Objs') - 3*eye(size(Objs, 1));
    [~, SInd] = sort(-CosV, 2);
    Neighbor = SInd(:, 1:10);
    P = arrayfun(@(i) Neighbor(i, randsample(10, 2)), (1:size(Objs, 1))', 'UniformOutput', false);
    P = cell2mat(P);
    Mat1 = Population(P(:, 1));
    Mat2 = Population(P(:, 2));
end
function Population = Update_time(Population,data,epsilon)
    for i = 1:length(Population)
        seq_sat = Population(1,i).add{1,1}{1,1}{1,1};
        route = Population(1,i).add{1,1}{1,1}{2,1};
        New_time = time_violate(seq_sat,route,data,data.vrp2e);
        if New_time > epsilon
            Population(1,i).con(5) = Population(1,i).con(5)-Population(1,i).con(4)+New_time;
            Population(1,i).con(4) = New_time;
        end
    end
end

