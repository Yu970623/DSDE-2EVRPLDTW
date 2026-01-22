classdef CMOEAD < ALGORITHM
% <multi/many> <real/integer/label/binary/permutation> <constrained/none>
% Constraint-MOEA/D

%------------------------------- Reference --------------------------------
% H. Jain and K. Deb, An evolutionary many-objective optimization algorithm
% using reference-point based non-dominated sorting approach, part II:
% Handling constraints and extending to an adaptive approach, IEEE
% Transactions on Evolutionary Computation, 2014, 18(4): 602-622.
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
            %% Generate the weight vectors
            [W,Problem.N] = UniformPoint(Problem.N,Problem.M);
            T  = ceil(Problem.N/10);
            nr = ceil(Problem.N/100);

            %% Detect the neighbours of each solution
            B = pdist2(W,W);
            [~,B] = sort(B,2);
            B = B(:,1:T);

            %% Generate random population
            Population = Problem.Initialization();
            Z = min(Population.objs,[],1);
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
                    fileName1 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\CMOEAD_%s_HV.mat', problem_name);
                    save(fileName1, 'I1', '-V7.3'); % 保存第一个文件
                    fileName2 = sprintf('F:\\Onedrive\\Experiment\\Test\\2EVRPLDTW-6\\Cov\\CMOEAD_%s_CV.mat', problem_name);
                    save(fileName2, 'c', '-V7.3'); % 保存第一个文件
                end
                % For each solution
                for i = 1 : Problem.N      
                    % Choose the parents
                    if rand < 0.9
                        P = B(i,randperm(size(B,2)));
                    else
                        P = randperm(Problem.N);
                    end

                    % Generate an offspring
                    Offspring = OperatorGAhalf(Problem,Population(P(1:2)));

                    % Update the ideal point
                    Z = min(Z,Offspring.obj);

                    % Calculate the constraint violation of offspring and P
                    CVO = sum(max(0,Offspring.con));
                    CVP = sum(max(0,Population(P).cons),2);

                    % Update the solutions in P by PBI approach
                    normW   = sqrt(sum(W(P,:).^2,2));
                    normP   = sqrt(sum((Population(P).objs-repmat(Z,length(P),1)).^2,2));
                    normO   = sqrt(sum((Offspring.obj-Z).^2,2));
                    CosineP = sum((Population(P).objs-repmat(Z,length(P),1)).*W(P,:),2)./normW./normP;
                    CosineO = sum(repmat(Offspring.obj-Z,length(P),1).*W(P,:),2)./normW./normO;
                    g_old   = normP.*CosineP + 5*normP.*sqrt(1-CosineP.^2);
                    g_new   = normO.*CosineO + 5*normO.*sqrt(1-CosineO.^2);
                    Population(P(find(g_old>=g_new & CVP==CVO | CVP>CVO,nr))) = Offspring;
                end
            end
        end
    end
end