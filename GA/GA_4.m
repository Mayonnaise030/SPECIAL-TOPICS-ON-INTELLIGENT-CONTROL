%% GA
%% 清空環境變數
clc,clear,close all
%% 設定system 和 Initialize
global y
y = 0;
% Define error
global err_sum
global pre_err
%err_sum = 0;
pre_err = 0;

% The command is a discrete time signal, magnitude=1
global command
command = ones(1, 240);
for period = 1:4
    for idx = 1:60
        if idx>30
            command(1, (period-1)*60+idx) = 0;
        end
    end
end

%% Find PID param
disp('Find PID param')
PID_param = op_GA()
%% 設定system 和 Initialize
global y
y = 0;
% Define error
err_sum = 0;
pre_err = 0;

% The command is a discrete time signal, magnitude=1
command = ones(1, 240);
for period = 1:4
    for idx = 1:60
        if idx>30
            command(1, (period-1)*60+idx) = 0;
        end
    end
end
%% Test the best PID param
disp('Test PID param')
result = System(PID_param);
% Print Result
figure;
t = 1:1:240;
plot(t, result, t , command);
xlabel('Iteration');
ylabel('magnitude');
legend('track' , 'command')
%% System function
function result = System(PID_param)
    global y
    global command
    global input_array
    global y_array
    for run = 1:240
        % recording array index
        if run == 1 
            % there is no feedback in first run, so let y_result = 0 
            err = command(1,run);
        else
            err = command(1,run) - y_result;
        end
%         disp(['get err = ' num2str(err)])
        plant_input = Controller(err, PID_param);
        input_array(1,run) = plant_input;

        y_result = Plant(plant_input);
        y_array(1, run) = y_result;
    end
    result = y_array;
end
%% Controller function
function plant_input = Controller(err, PID_param)
    global err_sum
    global pre_err
    err_sum = err_sum + err;
    
    % PID_param = [kp, ki, kd]
    plant_input = PID_param(1)*err + PID_param(2)*err_sum + PID_param(3)*(err-pre_err); 
    % PD_param = [kp, kd]
%     plant_input = PID_param(1)*err + PID_param(2)*(err-pre_err);
    
    pre_err = err;
end

%% Plant function
function y_result = Plant(plant_input)
    global y
    input_bound = 20;
    if abs(plant_input) > input_bound
        if plant_input>0
            plant_input = input_bound;
        elseif plant_input<0
            plant_input = -input_bound;
        end
    end
%     disp(['y = ' num2str(y)])
%     disp(['plant_input = ' num2str(plant_input)])
    y_result =  y /(1+y.^2) + plant_input.^3;
    y = y_result;
end
%% GA設定環境變數和初始化
function PID_param = op_GA()
    global err_sum
    global pre_err
    err_sum = 0;
    pre_err = 0;
    disp('op_GA')
    epoch = 4;
    pop_size = 20;                                                              % 染色體數量(poppulation)
    DNA_size = 3;                                                              % 染色體節點數(可以想看成一條染色體有多少區段，其實就是維度)
    max_iter = 60;                                                             % 迭代次數(可以理解為有幾代)
    %cross_rate = rand(1,1);                                                    % 交配率，介於0和1之間
    %mutation_rate = rand(1,1);                                                 % 突變率，介於0和1之間
    cross_rate = 0.5;                                                           % 交配率，定值較好
    mutation_rate = 0.2;                                                        % 突變率，定值較好
    varMin = 0;                                                                 % Var Lower Bound
    varMax = 0.4;                                                               % Var Upper Bound
    %染色體設定
    Gene = zeros(pop_size, DNA_size);                                           %創造一空染色體空間存放染色體
    Gene_range = [varMax; varMin];                                              %存放染色體區段的上下界
    fitness = zeros(pop_size, 1);                                               %存放染色體適應值
    ave_fitness = zeros(1, max_iter);                                           %記錄每一代平均適應值
    best_fitness = zeros(1, max_iter);                                          %記錄每一代最佳適應值
    G_best = zeros(1, DNA_size+1);                                              %記錄最佳染色體和最佳適應值
    %% Initialize
    % 開始迭代
    for t =1:epoch
        disp([num2str(t) ' epoch'])
        Gene = Initialize(pop_size, DNA_size, Gene_range);                          %初始化染色體
        fitness = Fitness(Gene, pop_size);                      %計算適應值
        G_best = Best(Gene, fitness, DNA_size);                                     %尋找最佳染色體和最佳適應值
        %G_best = min(fitness)                                                      %尋找最佳染色體和最佳適應值
        best_fitness(1) = G_best(end);                                              %存取當代最佳適應值
        ave_fitness(1) = Ave_Fitness(pop_size,fitness);
        for iter = 2:max_iter
            [Gene, fitness] = SELECT(Gene, G_best, fitness);
            % 突變
            Gene = MUTATION(Gene, mutation_rate,pop_size,DNA_size, Gene_range,iter,max_iter);   
            %交叉
            Gene = CROSS(Gene, cross_rate, pop_size,DNA_size);
            %重新計算適應值
            fitness = Fitness(Gene, pop_size);   
            %尋找最佳染色體和最佳適應值
            G_best_temp = Best(Gene, fitness, DNA_size);                            %尋找當代最佳染色體和最佳適應值
            if G_best_temp(end) < G_best(end)                                       %比較(> 表示找最大解 < 則相反)
               G_best = G_best_temp;
            end
            %進化                 
            best_fitness(iter) = G_best(end);                                       %存取當代最佳適應值
            %best_fitness(iter) = min(fitness)
            if t == 25 & iter == max_iter/2
                Medium = best_fitness(1,iter);
            end
            ave_fitness(iter) = Ave_Fitness(pop_size,fitness);                      %存取當代平均適應值
        end
    end
    [value, ii] = min(abs(G_best(:)-0));
    [row, col] = ind2sub(size(G_best), ii);
    disp(['最佳適應值為:']); disp(G_best(row, col));
    PID_param = G_best(1:3);
end

function Init = Initialize(pop_size, DNA_size, Gene_range)
% 創造一空染色體空間存放染色體並隨機放入0~1值
    Init = rand(pop_size, DNA_size);
    for i = 1:pop_size
        for j = 1:DNA_size 
        % 0~1值 * range + abs(range / 2)
        %舉實際例子-500~500 和rand → rand = rand * (500 - (-500))  
        Init(i, j) = Init(i, j)*(Gene_range(2)-Gene_range(1))+Gene_range(1);
        if j == 3
            Init(i, j) = Init(i, j)*0.02;
        else
            Init(i, j) = Init(i, j) + 0.1;
        end
        end
    end
end

function fitness = Fitness(Gene, pop_size)
    global command
    global err_sum
    err_sum = 0;
    fitness = zeros(pop_size, 1);
    %適應值計算
    for i = 1:pop_size
          PID_param = Gene(i,:);
          result = System(PID_param);
          for r = 1:size(result)
            fitness(i,1) = fitness(i,1) + (result(r) - command(r))^2;
          end 
    end
end

function [Gene_new, fitness_new] = SELECT(Gene, G_best, fitness)
    max_num = max(fitness);
    min_num = min(fitness);
    sum_num = sum(fitness);
    limit = (max_num-min_num)*0.5+min_num;                          %設定一值
    replace_gene = fitness > limit;                                 %找出大於該值的所有fintess點(這裡找最接近0所以是大於)
    replace_num = sum(replace_gene);                                
    %用目前最好的值取代大於該值的所有點
    Gene(replace_gene, :) = ones(replace_num, 1)*G_best(1:end-1);
    fitness(replace_gene) = ones(replace_num, 1)*G_best(end);
    Gene_new = Gene;
    fitness_new = fitness;
end

function ave_fitness = Ave_Fitness(pop_size,fitness)
    ave_fitness = sum(fitness)/pop_size;
end

function Generation_best = Best(Gene, fitness, DNA_size);
    Generation_best = zeros(1,DNA_size+1);
    [Best_fitness_value, Best_list] = min(fitness);
    Generation_best(1:DNA_size) =Gene(Best_list, :);
    Generation_best(end) = Best_fitness_value;
end

function Gene = MUTATION(Gene, mutation_rate,pop_size,DNA_size, Gene_range,iter,max_iter)
    for i = 1:pop_size
        for j = 1:DNA_size  
            mutation_rand = rand; %產生自然突變率
            if mutation_rand <= mutation_rate  %mutation_rate介於0和1之間含自然突變率想比，自然突變率小於mutation_rate才進行突變
                mutation_standard = rand; %增加還是減少
                mutation_num = rand*(1-iter/max_iter)^2;
                if mutation_standard <= 0.5
                    Gene(i, j)= Gene(i, j)*(1-mutation_num);
                else
                    Gene(i, j)= Gene(i, j)*(1+mutation_num);
                end
                %檢查有沒有超過範圍
                if Gene(i, 3) > 1
                    Gene(i, 3) = 1;
                end
                if Gene(i, j) >= Gene_range(1)
                    Gene(i, j) = Gene_range(1);
                elseif Gene(i, j) < Gene_range(2)
                    Gene(i, j) = Gene_range(2);
                else
                    Gene(i, j) = Gene(i, j);
                end
            end
        end
    end
end

function  Gene_new = CROSS(Gene, cross_rate, pop_size,DNA_size)
    for i = 1:pop_size
        cross_rand = rand;%產生自然交配率
        if cross_rand < cross_rate % cross_rate介於0和1之間含自然交配率相比，自然交配率小於cross_rate才進行突變
            cross_Gene = floor((pop_size-1)*rand+1); %要交配的染色體
            cross_node = floor((DNA_size-1)*rand+1); %要交配的染色體區段
            cross_node_2 = floor((DNA_size-1)*rand+1); %要交配的染色體區段
            %交配
            temp = Gene(i, cross_node);
            Gene(i, cross_node) = Gene(cross_Gene, cross_node); 
            Gene(cross_Gene, cross_node) = temp;
        end
        if Gene(i, 3) > 1
           Gene(i, 3) = 1;
        end
    end
    Gene_new = Gene;
end