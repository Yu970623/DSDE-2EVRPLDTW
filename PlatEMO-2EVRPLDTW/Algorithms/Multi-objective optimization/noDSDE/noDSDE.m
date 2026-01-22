classdef noDSDE < ALGORITHM
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
            Population = Problem.Initialization();
            Archive = ArchiveUpdate(Population,Problem.N);
            [W,Problem.N] = UniformPoint(Problem.N,Problem.M);
            Zmin    = min(Population.objs,[],1);
            epsilon_0 = max(overall_cv(Population.cons));
            Tc = 0.8 * ceil(Problem.maxFE/Problem.N);
            cp = 2;
            %% Optimization
            while Algorithm.NotTerminated(Population)
                gen = ceil(Problem.FE/Problem.N);
                if Problem.maxFE<=Tc
                    epsilon = epsilon_0 * ((1 - (gen / Tc)) ^ cp);
                else
                    epsilon = 0;
                end
                MatingPool = TournamentSelection(2,Problem.N,sum(max(0,Population.cons),2));
                [Mat1,Mat2] = Neighbor_Sort(Population(MatingPool),Zmin);
                Offspring = OperatorDE(Problem,Population(MatingPool),Mat1,Mat2,{rand(),rand(),rand(),rand()});
                Zmin       = min([Zmin;Offspring.objs],[],1);
                [Population,~] = EnvironmentalSelection([Population,Offspring],Problem.N,W,Zmin,epsilon);
                Archive = ArchiveUpdate([Archive,Population,Offspring],Problem.N);
            end
        end
    end
end
function result = overall_cv(cv)
    cv(cv <= 0) = 0;cv = abs(cv);
    result = sum(cv,2);
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

