algorithms = {'DSDE_Courier','DSDE','DSDE_Courier'};
Compare_algo = cell(1,length(algorithms));
num_set = {'Ca1','Ca2','Ca3','Ca4','Ca5'};
num_depsat = {'23','35','64'};
num_cus = {'15','30','50','100'};
Objs = zeros(60,8,2);
mean_objs = zeros(5,3);  %
complete = zeros(60,3);
for i = 1:2  %算法文件夹
    for l = 1:4   %客户规模 
        for m = 1:5  %Ca 12345
            for k = 1:3
                for j = 1:8  %重复运行次数
                    algo_name = algorithms{i};
                    load(['F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data\',algo_name,'\','DSDE_',num_set{m},'_',num_depsat{k},'_',num_cus{l},'_TW_M2_D10','_',num2str(j),'.mat']);
                    totla_obj = 0;
                    for n = 1:20
                        totla_obj = totla_obj + sum(result{1,2}(1,n).obj);
                    end
                    Objs((l-1)*15 + (m-1)*3 + k,j,i) = totla_obj/20;
                end
            end
        end
    end
    mean_objs(1:4, i) = sum(reshape(sum(Objs(:, :, i), 2), 15, 4), 1)' ./ (15 * 8);
    complete(1:60,i) = sum(Objs(:, :, i), 2)./ 8;
    mean_objs(5,i) = sum(mean_objs(1:4,i));
end

mean_objs(1:5, 3) = abs((mean_objs(1:5, 2) - mean_objs(1:5, 1)) ./ mean_objs(1:5, 1));
complete(1:60, 3) = abs((complete(1:60, 2) - complete(1:60, 1)) ./ complete(1:60, 1));


% 将结果矩阵写入Excel文件
filename1 = 'F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data_handle\Courier_Drone.xlsx';
writematrix(mean_objs, filename1);
filename2 = 'F:\Onedrive\Experiment\Test\2EVRPLDTW-6\Data_handle\Courier_Drone_full.xlsx';
writematrix(complete, filename2);