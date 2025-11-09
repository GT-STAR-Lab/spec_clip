clear; close all; clc;

algorithms_path = 'algorithms/'; % path for stable LDS algorithms
save_directory = 'results_kodex/';
data_directory = 'datasets/kodex';

mkdir(save_directory);

status = 0;

addpath(algorithms_path);
options.graphic = 0;
options.posdef = 10e-12;
options.maxiter = 200000;


data_paths = {
    % 'relocation_demo.mat', 
    % 'relocation_failed.mat', 
    % 'tool_original.mat', 
    % 'tool_truncated_to_40.mat',
    % 'tool_truncated_to_31.mat',
    % 'tool_truncated_to_35.mat',
    'door.mat',
    };

for i = 1:length(data_paths)
    name = data_paths{i}(1:end-4);
    load(fullfile(data_directory,data_paths{i}));

    timeLS = tic;
    LS = LDS(X.', Y.');
    stored_var = whos();
    memory_used_LS = 0;
    for i = 1 : length(stored_var)
        memory_used_LS = memory_used_LS + stored_var(i).bytes;
    end
    tLS = toc;

    alg_name = 'ls';
    experiment_name = [name, '_', alg_name, '_'];

    save([save_directory, experiment_name, 'time', '.mat'], "tLS");
    save([save_directory, experiment_name, 'mem', '.mat'], "memory_used_LS");    
    save([save_directory, experiment_name, 'matrix', '.mat'], "LS");

    disp('LS SAVED');
    
    % timeSC = tic;
    % LS = LDS(X.', Y.');
    % SC = eigenclip(LS, 1);
    % memory_used_SC = 0;
    % for i = 1 : length(stored_var)
    %     memory_used_SC = memory_used_SC + stored_var(i).bytes;
    % end
    % tSC = toc(timeSC);
    % 
    % alg_name = 'SC';
    % experiment_name = [name, '_', alg_name, '_'];
    % 
    % save([save_directory, experiment_name, 'time', '.mat'], "tSC");
    % save([save_directory, experiment_name, 'mem', '.mat'], "memory_used_SC");    
    % save([save_directory, experiment_name, 'matrix', '.mat'], "SC");
    % 
    % disp('SC SAVED');
    % 
    % alg_name = 'soc';
    % experiment_name = [name, '_', alg_name, '_'];
    % 
    % X_ = X.';
    % Y_ = Y.';
    % 
    % timeSOC = tic;
    % [SOC, memory_used_SOC] = learnSOCmodelKnowingLS(X_, Y_, LS, options);
    % tSOC = toc(timeSOC);
    % 
    % save([save_directory, experiment_name, 'time', '.mat'], "tSOC");
    % save([save_directory, experiment_name, 'mem', '.mat'], "memory_used_SOC");    
    % save([save_directory, experiment_name, 'matrix', '.mat'], "SOC");
    % 
    % disp('SOC SAVED');

end