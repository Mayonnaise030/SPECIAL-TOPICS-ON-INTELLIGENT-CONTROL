clc,clear,close all

disp('PSO_2')
%% Initialize
% Initialize y
global y
y = 2;

% Define error
global err_sum
global pre_err
err_sum = 0;
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

% Recording Result Array, length=240
global input_array
% global y_array
input_array = ones(1, 240);
% y_array = ones(1, 240);

%% Find PID param
disp('Find PID param')
tic;
PID_param = op_PSO();
PID_param
toc;
%% Test the best PID param
disp('Test PID param')
result = System(PID_param);
% Print Result
figure;
t = 1:1:240;
plot(t, result, t , command);

% % result = System(PID_param);

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

        if run <= 4 || command(1,run) ~= command(1,run-4)
            post_input = plant_input;
        else 
            post_input = input_array(1,run-4);
        end
        y_result = Plant(post_input);
        y_array(1, run) = y_result;
    end
%     disp(input_array)
    result = y_array;
end
%% Controller function
function plant_input = Controller(err, PID_param)
    global err_sum
    global pre_err
    input_bound = 20;
    err_sum = err_sum + err;
    
    % PID_param = [kp, ki, kd]
    plant_input = PID_param(1)*err + PID_param(2)*err_sum + PID_param(3)*(err-pre_err); 
    % PD_param = [kp, kd]
%     plant_input = PID_param(1)*err + PID_param(2)*(err-pre_err);

    pre_err = err;
%     
%     if abs(plant_input) > input_bound
%         if plant_input>0
%             plant_input = input_bound;
%         elseif plant_input<0
%             plant_input = -input_bound;
%         end
%     end
end

%% Plant function
% function y_result = Plant(post_input)
%     global y
%     c = 0.2;
% 
%     y_result = 0.95 * y + c * post_input;
%     y = y_result;
% end

function y_result = Plant(plant_input)
    global y
    c_2 = 0.1;
    input_bound = 20;
    if abs(plant_input) > input_bound
        if plant_input>0
            plant_input = input_bound;
        elseif plant_input<0
            plant_input = -input_bound;
        end
    end
    
    y_result = 0.95 * y + c_2 * plant_input;
    y = y_result;
end

%% Optimizer function
function PID_param = op_PSO()
    global err_sum
    global pre_err
    err_sum = 0;
    pre_err = 0;
    epoch = 5;
    Iteration = 500;             
    population = 30;             % particle number   
    dim = 3;                    % dimension  

    w = 0.8;                     % weight
    c1 = 1;                      % acceleration constants
    c2 = 1;                      % acceleration constants
    r1 = 0.1;                    % random r1
    r2 = 0.1;                    % random r2

    bestness = zeros(Iteration, 1);
    tic;
    ave_value = 0; 
    for r = 1 : epoch
        disp([num2str(r) ' epoch'])
        % initialize
        particle.pos = ones(1, dim);  % personal position
        particle.vel = [];            % personal velocity
        particle.cost = [];           % personal cost
        p_best_particle.pos = [];     % personal best position
        p_best_particle.cost = [];    % personal best cost 
        g_best.pos = [];              % global best position 
        g_best.cost = inf;            % global best cost

        pop = repmat(particle, population, 1);
        p_best = repmat(p_best_particle, population, 1);
        for i = 1:population
            % Initialize PID_param
            pop(i).pos(1) = rand(1, 1)*2; 
            pop(i).pos(2) = rand(1, 1)*0.1; % ki
            pop(i).pos(3) = rand(1, 1)*0.1; % kd
            pop(i).vel = zeros(1, dim);
            pop(i).cost = cost_function(pop(i).pos); 
            p_best(i).pos = pop(i).pos;  
            p_best(i).cost = pop(i).cost;  
            if p_best(i).cost < g_best.cost
                g_best.pos = p_best(i).pos; 
                g_best.cost = p_best(i).cost;  
            end
        end

        % Update
        disp('  Update ')
        for t = 1:Iteration  
%         disp([num2str(t) ' Iteration'])
            for i = 1:population  
                % update velocity
                pop(i).vel = w*pop(i).vel + c1*r1*(p_best(i).pos - pop(i).pos) + c2*r2*(g_best.pos - pop(i).pos);
                % velocity limit %
                % update position           
                pop(i).pos = pop(i).pos + pop(i).vel;  
            end
            % update global best??? personal best
            for i = 1:population                              
               pop(i).cost = cost_function(pop(i).pos); 
               ave_value = ave_value + pop(i).cost;
               % update personal best 
               if pop(i).cost < p_best(i).cost                
                   p_best(i).pos = pop(i).pos;  
                   p_best(i).cost = pop(i).cost; 
                   % update global best
                   if p_best(i).cost < g_best.cost             
                       g_best.pos = p_best(i).pos;  
                       g_best.cost = p_best(i).cost;  
                   end
               end
            end
            bestness(t) = bestness(t) + g_best.cost;
        end
    end
    PID_param = g_best.pos;
end
    
% Cost Function
function cost = cost_function(pos)
    global command
    global err_sum
    err_sum = 0;
    
    PID_param = pos;
    result = System(PID_param);
    cost = 0;
    for i = 1:size(result)
%         cost = cost + (result(i) - command(i))^2;
        cost = (cost*(i-1) + abs(result(i) - command(i))) / i;
    end
end