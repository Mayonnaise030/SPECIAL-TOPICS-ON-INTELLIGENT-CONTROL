clc,clear,close all

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
input_array = ones(1, 240);

%% Test the best PID param
disp('Test PID param')
result = System([1.0492 0.099 0.0501]);
% Print Result2.5103    0.0238    0.0240
figure;
t = 1:1:240;
plot(t, result, t, command);


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
        
        plant_input = Controller(err, PID_param);
        input_array(1,run) = plant_input;
        
        %%% 2
        if run <= 4 || command(1,run) ~= command(1,run-4)
            post_input = plant_input;
        else 
            post_input = input_array(1,run-4);
        end
        plant_input = post_input;
%         %%% 3
%         % Get u(k+1)
%         y_result = Plant(plant_input);
%         err = command(1,run) - y_result;
%         pre_input = Controller(err, PID_param);
%         plant_input = pre_input;
        
        y_result = Plant(plant_input);
        y_array(1, run) = y_result;
    end
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
        
    pre_err = err;
    
%     if abs(plant_input) > input_bound
%         if plant_input>0
%             plant_input = input_bound;
%         elseif plant_input<0
%             plant_input = -input_bound;
%         end
%     end
end

%% Plant function 1
% function y_result = Plant(plant_input)
%     global y
%     a = 0.3;
%     b = 0.1;
%     
%     y_result = a * y + b * y^3 + 0.2 * plant_input;
%     y = y_result;
% end

%% Plant function 2
% function y_result = Plant(post_input)
%     global y
%     c = 0.3;
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

%% Plant function 3
% function y_result = Plant(pre_input)
%     global y
%     c = 0.05;
% 
%     y_result = y + c * pre_input;
%     y = y_result;9
% end

%% Plant function 4
% function y_result = Plant(plant_input)
%     global y
% 
%     y_result = y/(1+y^2) + plant_input^3;
%     y = y_result;
% end