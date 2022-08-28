%% GA
%% �M�������ܼ�
clc,clear,close all
%% �]�wsystem �M Initialize
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
%% �]�wsystem �M Initialize
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
%% GA�]�w�����ܼƩM��l��
function PID_param = op_GA()
    global err_sum
    global pre_err
    err_sum = 0;
    pre_err = 0;
    disp('op_GA')
    epoch = 4;
    pop_size = 20;                                                              % �V����ƶq(poppulation)
    DNA_size = 3;                                                              % �V����`�I��(�i�H�Q�ݦ��@���V���馳�h�ְϬq�A���N�O����)
    max_iter = 60;                                                             % ���N����(�i�H�z�Ѭ����X�N)
    %cross_rate = rand(1,1);                                                    % ��t�v�A����0�M1����
    %mutation_rate = rand(1,1);                                                 % ���ܲv�A����0�M1����
    cross_rate = 0.5;                                                           % ��t�v�A�w�ȸ��n
    mutation_rate = 0.2;                                                        % ���ܲv�A�w�ȸ��n
    varMin = 0;                                                                 % Var Lower Bound
    varMax = 0.4;                                                               % Var Upper Bound
    %�V����]�w
    Gene = zeros(pop_size, DNA_size);                                           %�гy�@�ŬV����Ŷ��s��V����
    Gene_range = [varMax; varMin];                                              %�s��V����Ϭq���W�U��
    fitness = zeros(pop_size, 1);                                               %�s��V����A����
    ave_fitness = zeros(1, max_iter);                                           %�O���C�@�N�����A����
    best_fitness = zeros(1, max_iter);                                          %�O���C�@�N�̨ξA����
    G_best = zeros(1, DNA_size+1);                                              %�O���̨άV����M�̨ξA����
    %% Initialize
    % �}�l���N
    for t =1:epoch
        disp([num2str(t) ' epoch'])
        Gene = Initialize(pop_size, DNA_size, Gene_range);                          %��l�ƬV����
        fitness = Fitness(Gene, pop_size);                      %�p��A����
        G_best = Best(Gene, fitness, DNA_size);                                     %�M��̨άV����M�̨ξA����
        %G_best = min(fitness)                                                      %�M��̨άV����M�̨ξA����
        best_fitness(1) = G_best(end);                                              %�s����N�̨ξA����
        ave_fitness(1) = Ave_Fitness(pop_size,fitness);
        for iter = 2:max_iter
            [Gene, fitness] = SELECT(Gene, G_best, fitness);
            % ����
            Gene = MUTATION(Gene, mutation_rate,pop_size,DNA_size, Gene_range,iter,max_iter);   
            %��e
            Gene = CROSS(Gene, cross_rate, pop_size,DNA_size);
            %���s�p��A����
            fitness = Fitness(Gene, pop_size);   
            %�M��̨άV����M�̨ξA����
            G_best_temp = Best(Gene, fitness, DNA_size);                            %�M���N�̨άV����M�̨ξA����
            if G_best_temp(end) < G_best(end)                                       %���(> ��ܧ�̤j�� < �h�ۤ�)
               G_best = G_best_temp;
            end
            %�i��                 
            best_fitness(iter) = G_best(end);                                       %�s����N�̨ξA����
            %best_fitness(iter) = min(fitness)
            if t == 25 & iter == max_iter/2
                Medium = best_fitness(1,iter);
            end
            ave_fitness(iter) = Ave_Fitness(pop_size,fitness);                      %�s����N�����A����
        end
    end
    [value, ii] = min(abs(G_best(:)-0));
    [row, col] = ind2sub(size(G_best), ii);
    disp(['�̨ξA���Ȭ�:']); disp(G_best(row, col));
    PID_param = G_best(1:3);
end

function Init = Initialize(pop_size, DNA_size, Gene_range)
% �гy�@�ŬV����Ŷ��s��V������H����J0~1��
    Init = rand(pop_size, DNA_size);
    for i = 1:pop_size
        for j = 1:DNA_size 
        % 0~1�� * range + abs(range / 2)
        %�|��ڨҤl-500~500 �Mrand �� rand = rand * (500 - (-500))  
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
    %�A���ȭp��
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
    limit = (max_num-min_num)*0.5+min_num;                          %�]�w�@��
    replace_gene = fitness > limit;                                 %��X�j��ӭȪ��Ҧ�fintess�I(�o�̧�̱���0�ҥH�O�j��)
    replace_num = sum(replace_gene);                                
    %�Υثe�̦n���Ȩ��N�j��ӭȪ��Ҧ��I
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
            mutation_rand = rand; %���ͦ۵M���ܲv
            if mutation_rand <= mutation_rate  %mutation_rate����0�M1�����t�۵M���ܲv�Q��A�۵M���ܲv�p��mutation_rate�~�i�����
                mutation_standard = rand; %�W�[�٬O���
                mutation_num = rand*(1-iter/max_iter)^2;
                if mutation_standard <= 0.5
                    Gene(i, j)= Gene(i, j)*(1-mutation_num);
                else
                    Gene(i, j)= Gene(i, j)*(1+mutation_num);
                end
                %�ˬd���S���W�L�d��
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
        cross_rand = rand;%���ͦ۵M��t�v
        if cross_rand < cross_rate % cross_rate����0�M1�����t�۵M��t�v�ۤ�A�۵M��t�v�p��cross_rate�~�i�����
            cross_Gene = floor((pop_size-1)*rand+1); %�n��t���V����
            cross_node = floor((DNA_size-1)*rand+1); %�n��t���V����Ϭq
            cross_node_2 = floor((DNA_size-1)*rand+1); %�n��t���V����Ϭq
            %��t
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