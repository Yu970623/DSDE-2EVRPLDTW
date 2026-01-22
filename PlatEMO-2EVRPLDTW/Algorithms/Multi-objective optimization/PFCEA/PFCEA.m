classdef PFCEA < ALGORITHM
% <multi> <real> <constrained>

    methods
        function main(Algorithm,Problem)
            
            %% Parameter setting
            [delta,nr] = Algorithm.ParameterSet(0.9,2);

            max_change       = 1;
            last_gen         = 20;
            change_threshold = 1e-2;
            ideal_points     = zeros(ceil(Problem.maxFE/Problem.N),Problem.M);
            nadir_points     = zeros(ceil(Problem.maxFE/Problem.N),Problem.M);
            %% Evaluate the Population
             rmp = 0.8;      
             Nt = 2 ;      % The number of tasks
             Beta = 0.2;   % Learning phase
             eta  = 0.4;
             gama = 0.5;    %$
             ff = 0 ;    %$
             Pmin = 0.1;   % Minimum selection probability
             
            %% Generate the weight vectors
             [W,Problem.N] = UniformPoint(Problem.N,Problem.M);
             T = ceil(Problem.N/10);
            
            %% Detect the neighbours of each solution
             B = pdist2(W,W);
             [~,B] = sort(B,2);
             B = B(:,1:T);
            
            %% Initialization 
             Population{1} = Problem.Initialization();
             Population{2} = Problem.Initialization();
             Z{1} = min(Population{1}.objs,[],1);
             Conmin = min(overall_cv(Population{1}.cons));
             Z{2} = min(Population{2}.objs,[],1);
             A = Population{1}; 
             rwd   =  zeros(Nt, 1);                 % Rewards

             totalcon = size(Population{1}(1,1).con,2);
             precon = 0;
            I1 = [];
            S1 = HV(A,Problem.optimum);
            I1 = [I1,S1];
            %% Optimization
             while Algorithm.NotTerminated(A) 
                if mod(Problem.FE,Problem.N)==0
                    S = HV(A,Problem.optimum);
                    I1 = [I1,S];
                end
                if Problem.FE >= Problem.maxFE-Problem.N
                    problem_name = class(Problem); % 获取类名
                    fileName1 = sprintf('F:\\Onedrive\\Experiment\\2EVRPLDTW\\Cov\\PFCEA_%s_HV.mat', problem_name);
%                     save(fileName1, 'I1', '-V7.3'); % 保存第一个文件
                end
                CV    = sum(max(Population{1}.cons,0),2);
                fr    = length(find(CV<=0))/Problem.N;    
                sigma_obj = 0.3;
                sigma_cv = fr*10;
                Q = [];
                iter = 2.718*Problem.FE/Problem.maxFE;
                cv    = sum(max(Population{2}.cons,0),2);
                gen        = ceil(Problem.FE/Problem.N);
                population = [Population{2}.decs,Population{2}.objs,cv];
                ideal_points(gen,:) = Z{2};
                nadir_points(gen,:) = max(population(:,Problem.D + 1 : Problem.D + Problem.M),[],1);

                % The maximumrate of change of ideal and nadir points rk is calculated.
                if gen >= last_gen
                    max_change = calc_maxchange(ideal_points,nadir_points,gen,last_gen);
                end

                % Calculate the selection probability 
                if Problem.FE <= 0.5 * Problem.maxFE
                    % Stage 1: Evolution stage
                    pro  =  1 / Nt * ones(1, Nt);   % Selection probability
                    
                else
                    % Stage 2: Competition stage
                    if sum(rwd) ~= 0
                        pro   =   Pmin / Nt + (1 - Pmin) * rwd ./ sum(rwd);
                        pro   =   pro ./ sum(pro);
                    else
                        pro   =   1 / Nt * ones(1, Nt);
                    end
                end
                if (Problem.FE >= Beta * Problem.maxFE && ff == 0 && max_change <= change_threshold) || (Problem.FE >= eta * Problem.maxFE && ff == 0)
                    ff = 1;
                    maxCV = 0; maxtempCV = 0;
                    for i = 1:totalcon
                        temp = part_cv(Population{2}.cons,i);
                        numCV = sum(temp>0);
                        tempCV = sum(part_cv(Population{2}.cons,i),"all");
                        if numCV >=0.8*Problem.N
                            if tempCV > maxtempCV
                                precon = i ;
                                maxCV  = numCV;
                            end
                        end
                        if numCV > maxCV && precon < i && numCV >=0.2*Problem.N
                            precon = i;
                            maxCV = numCV;
                        end
                    end

                end

                % Determine the a task based on the selection probability using roulette wheel method
                r = rand;
                for t = 1:Nt
                    if r <= sum(pro(1:t))
                        k = t;
                        break;
                    end
                end
                
                % For each solution
                for i = 1:Problem.N
                    % Choose the parents
                    if rand < delta
                        P = B(i,randperm(end));
                    else
                        P = randperm(Problem.N);
                    end
    
                   %% IFCHT is the main task  
                    if k == 1   
                        % Generate an offspring
                        if rand < rmp
                            Offspring = OperatorDE(Problem,Population{k}(i),Population{k}(P(1)),Population{k}(P(2))); 
                        else
                            Offspring = OperatorDE(Problem,Population{k}(i),Population{2}(P(1)),Population{2}(P(2))); 
                        end
                        % Update the ideal point
                        Z{1} = min(Z{1},Offspring.obj);
                        Conmin = min(Conmin,overall_cv(Offspring.con));
                       
                        Conmax = max(overall_cv([Population{1}.cons;Offspring.con]));   
                        % Update the solutions in P by Tchebycheff approach
                        g_old = max(abs(Population{1}(P).objs-repmat(Z{1},length(P),1))./W(P,:),[],2);    
                        g_new = max(repmat(abs(Offspring.obj-Z{1}),length(P),1)./W(P,:),[],2);         
                        cv_old = overall_cv(Population{1}(P).cons);   
                        cv_new = overall_cv(Offspring.con);
                        if Conmax > Conmin
                            cv_old(cv_old > 0) = (cv_old(cv_old > 0)-Conmin)/(Conmax-Conmin);
                            cv_new(cv_new > 0) = (cv_new(cv_new > 0)-Conmin)/(Conmax-Conmin);    
                        end
                        if Problem.FE < gama*Problem.maxFE
                            new_old = LG(g_old - g_new,[sigma_obj 0]).*max(LG(cv_old - repmat(cv_new,length(P),1),[sigma_cv 0]),0.0001);   
                            old_new = LG(g_new - g_old,[sigma_obj 0]).*max(LG(repmat(cv_new,length(P),1) - cv_old,[sigma_cv 0]),0.0001);  
                        else
                            new_old = LG(g_old - g_new,[sigma_obj 0]).*max(LG((cv_old - repmat(cv_new,length(P),1))*iter,[sigma_cv 0]),0.0001);   
                            old_new = LG(g_new - g_old,[sigma_obj 0]).*max(LG((repmat(cv_new,length(P),1) - cv_old)*iter,[sigma_cv 0]),0.0001); 
                        end 
                        Population{1}(P(find(new_old>=old_new,nr))) = Offspring;
    
                    % for task2 - SCP
                        Z{2} = min(Z{2},Offspring.obj);
                        % Update the solutions in P by PBI approach
                        normW   = sqrt(sum(W(P,:).^2,2));
                        normP   = sqrt(sum((Population{2}(P).objs-repmat(Z{2},length(P),1)).^2,2));
                        normO   = sqrt(sum((Offspring.obj-Z{2}).^2,2));
                        CosineP = sum((Population{2}(P).objs-repmat(Z{2},length(P),1)).*W(P,:),2)./normW./normP;
                        CosineO = sum(repmat(Offspring.obj-Z{2},length(P),1).*W(P,:),2)./normW./normO;
                        g_old   = normP.*CosineP + 5*normP.*sqrt(1-CosineP.^2);
                        g_new   = normO.*CosineO + 5*normO.*sqrt(1-CosineO.^2);
                        
                        if ~precon
                            Population{2}(P(g_old>=g_new)) = Offspring;
                        else
                            cv_old = part_cv(Population{2}(P).cons,precon);
                            cv_new = part_cv(Offspring.con,precon);
                            Population{2}(P(find(((g_old >= g_new) & (((cv_old <= 0) & (cv_new <= 0)) | (cv_old == cv_new)) | (cv_new < cv_old) ), nr))) = Offspring;
                        end
                    end
                    
                   %% SCP is the main task
                    if k == 2  
                        % Generate an offspring
                        if rand < rmp
                            Offspring = OperatorGAhalf(Problem,Population{k}(P(1:2))); 
                        else
                            Offspring = OperatorGAhalf(Problem,Population{1}(P(1:2))); 
                        end
                        
                        % Update the ideal point
                        Z{2} = min(Z{2},Offspring.obj);
                        % Update the solutions in P by PBI approach
                        normW   = sqrt(sum(W(P,:).^2,2));
                        normP   = sqrt(sum((Population{2}(P).objs-repmat(Z{2},length(P),1)).^2,2));
                        normO   = sqrt(sum((Offspring.obj-Z{2}).^2,2));
                        CosineP = sum((Population{2}(P).objs-repmat(Z{2},length(P),1)).*W(P,:),2)./normW./normP;
                        CosineO = sum(repmat(Offspring.obj-Z{2},length(P),1).*W(P,:),2)./normW./normO;
                        g_old   = normP.*CosineP + 5*normP.*sqrt(1-CosineP.^2);
                        g_new   = normO.*CosineO + 5*normO.*sqrt(1-CosineO.^2);
                        if ~precon
                            Population{2}(P(g_old>=g_new)) = Offspring;
                        else
                            cv_old = part_cv(Population{2}(P).cons,precon);
                            cv_new = part_cv(Offspring.con,precon);
                            Population{2}(P(find(((g_old >= g_new) & (((cv_old <= 0) & (cv_new <= 0)) | (cv_old == cv_new)) | (cv_new < cv_old) ), nr))) = Offspring;
                        end 
                        
                    % for task1 - MOEAD-IFCHT
                        % Update the ideal point
                        Z{1} = min(Z{1},Offspring.obj);
                        Conmin = min(Conmin,overall_cv(Offspring.con));
                        
                        Conmax = max(overall_cv([Population{1}.cons;Offspring.con]));
                        % Update the solutions in P by Tchebycheff approach
                        g_old = max(abs(Population{1}(P).objs-repmat(Z{1},length(P),1))./W(P,:),[],2);    
                        g_new = max(repmat(abs(Offspring.obj-Z{1}),length(P),1)./W(P,:),[],2);         
                        cv_old = overall_cv(Population{1}(P).cons);   
                        cv_new = overall_cv(Offspring.con);
                        if Conmax > Conmin
                            cv_old(cv_old > 0) = (cv_old(cv_old > 0)-Conmin)/(Conmax-Conmin);
                            cv_new(cv_new > 0) = (cv_new(cv_new > 0)-Conmin)/(Conmax-Conmin);    
                        end
                        if Problem.FE < gama*Problem.maxFE
                            new_old = LG(g_old - g_new,[sigma_obj 0]).*max(LG(cv_old - repmat(cv_new,length(P),1),[sigma_cv 0]),0.0001);   
                            old_new = LG(g_new - g_old,[sigma_obj 0]).*max(LG(repmat(cv_new,length(P),1) - cv_old,[sigma_cv 0]),0.0001);  
                        else
                            new_old = LG(g_old - g_new,[sigma_obj 0]).*max(LG((cv_old - repmat(cv_new,length(P),1))*iter,[sigma_cv 0]),0.0001);   
                            old_new = LG(g_new - g_old,[sigma_obj 0]).*max(LG((repmat(cv_new,length(P),1) - cv_old)*iter,[sigma_cv 0]),0.0001); 
                        end   
                        Population{1}(P(find(new_old>=old_new,nr))) = Offspring;
                    end
                    Q = [Q Offspring];
                    
                end
                [Q, ~] =  A_Update([Population{k} Q],Problem.N);
                if size(Q,2) > 0
                    s = size(A,2);
                    [A,Next] =  A_Update([A Q],Problem.N);    % update Archive
                    if s >= Problem.N   
                        rwd(k)   =  rwd(k) + sum(Next(Problem.N+1:end))/Problem.N;   % update the reward of the main task
                    end
                end
                if Problem.FE < eta * Problem.maxFE
                    rwd   =  zeros(Nt, 1);
                end

             end
        end
    end  
end
function max_change = calc_maxchange(ideal_points,nadir_points,gen,last_gen)
    delta_value = 1e-6 * ones(1,size(ideal_points,2));
    rz = abs((ideal_points(gen,:) - ideal_points(gen - last_gen + 1,:)) ./ max(ideal_points(gen - last_gen + 1,:),delta_value));
    nrz = abs((nadir_points(gen,:) - nadir_points(gen - last_gen + 1,:)) ./ max(nadir_points(gen - last_gen + 1,:),delta_value));
    max_change = max([rz, nrz]);
end
function result = overall_cv(cv)
	cv(cv <= 0) = 0;cv = abs(cv);   
	result = sum(cv,2);            
end
function result = part_cv(cv,precon)
    cv(cv <= 0) = 0;
    cv = abs(cv(:,precon));  
	result = sum(cv,2); 
end
function y = LG(x, para)    
    if (x==0)
        y = repmat(0.5,length(x),1);  
    else
        y = (1+exp(-1*(x-para(2))./repmat(para(1),length(x),1))).^(-1);
    end
end
