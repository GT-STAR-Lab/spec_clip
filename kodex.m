%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Description
%
%       This file uses experimental data from the Franka manipulator to 
%       learn a LDS with inputs using different data-driven methods. In the 
%       end, it saves LQR gains for all data-driven models. 
%       The data-driven learning methods used are:
%           1. Least-squares (LS) unconstrained (possibly unstable) A and B 
%              matrix pair
%           2. A learned matrix pair [A, B] with SOC, that simultaneously 
%              learns a stable A, and a B matrix. 
%           3. A learned matirx pair [A, B] with WLS, that learns a stable 
%              A, without updating the least-squares B matrix solution. 
%           4. A learned matrix pair [A,B] with CG, that learns a stable A, 
%              without updating the least-squares B matrix solution.
%
%
%
%       Given experimental data from the Franka manipulator, the code:
%           1. Combines all data (discontinuous) runs into one file
%           2. Computes the least-squares solution
%           3. Computes the SOC, WLS, and CG stable solutions
%           4. Calculates LQR gains for each method
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc; system = 'Franka';

algorithms_path = 'algorithms/'; % path for stable LDS algorithms
save_directory = 'results_kodex/';
data_directory = 'prepare_data/kodex';

mkdir(save_directory);

status = 0;

addpath(algorithms_path);
options.graphic = 0;
options.posdef = 10e-12;
options.maxiter = 200000;


data_paths = {
    'relocation_demo.mat', 
    'relocation_failed.mat', 
    'tool_original.mat', 
    'tool_truncated_to_40.mat',
    'tool_truncated_to_31.mat',
    'tool_truncated_to_35.mat'};

for i = 1:length(data_paths)
    name = data_paths{i}(1:end-4);
    load(fullfile(data_directory,data_paths{i}));
    % load(['matrix_', data_paths{i}]);

    % X_ = X;
    % Y_ = Y;

    timeLS = tic;
    timeSC = tic;
    LS = LDS(X.', Y.');
    tLS = toc;

    SC = eigenclip(LS, 1);
    tSC = toc(timeSC);

    alg_name = 'ls';
    experiment_name = [name, '_', alg_name, '_'];

    save([save_directory, experiment_name, 'time', '.mat'], "tLS");
    save([save_directory, experiment_name, 'matrix', '.mat'], "LS");

    disp('LS SAVED');

    alg_name = 'SC';
    experiment_name = [name, '_', alg_name, '_'];

    save([save_directory, experiment_name, 'time', '.mat'], "tSC");
    save([save_directory, experiment_name, 'matrix', '.mat'], "SC");

    disp('SC SAVED');

    alg_name = 'soc';
    experiment_name = [name, '_', alg_name, '_'];

    X_ = X.';
    Y_ = Y.';

    timeSOC = tic;
    [SOC, ~] = learnSOCmodelKnowingLS(X_, Y_, LS, options);
    tSOC = toc(timeSOC);

    save([save_directory, experiment_name, 'time', '.mat'], "tSOC");
    save([save_directory, experiment_name, 'matrix', '.mat'], "SOC");

    disp('SOC SAVED');

end




% dataRoot = 'prepare_data\dtdb\prepared\';
% for seq = 0:30
%     disp(['current seq ', num2str(seq)]);
%     loc = [dataRoot, 'dtdb_', num2str(seq + 1), '.mat'];
%     data = double(load(loc).seq.') / 255.0;
%     [total_state, length] = size(data);
%     if(length > 300)
%         data = data(:, 1:300);
%     end
%     % Y = data(2:end, :);
%     % X = data(1:end - 1, :);
%     [data_U, data_S, data_V] = svd(data);
% 
% 
%     for stateNum = 3:30
%         SOCspace_S = data_S(1:stateNum, :);
%         SOCspace_data = SOCspace_S * data_V.';
%         disp(['dimension ', num2str(stateNum)])
%         X_ = SOCspace_data(:, 1:end - 1);
%         Y_ = SOCspace_data(:, 2:end);
% 
% 
%         %% Compute LS (unconstrained) [A, B] solution
%         fprintf('Computing LS unconstrained [A, B] solution ... \n');
%         randomShuffling = false;
%         tStart = tic;
%         [SC, LS] = LDS(X_, Y_);
%         tEnd = toc(tStart);
%         disp(tEnd);
%         cl_e_LS = norm(Y_ - SC * X_, 'fro')^2/2;
%         uncl_e_LS = norm(Y_ - LS * X_, 'fro')^2/2;
%         error(stateNum - 2) = uncl_e_LS;
%         maxeval_LS = max(abs(eig(SC)) );
%         maxeval_LS_unclipped = max(abs(eig(LS)) );
% 
%         if(maxeval_LS_unclipped <= 1)
%             unclip_stability(seq+1, stateNum - 2) = 1;
%         end
%         if(maxeval_LS <= 1)
%             clip_stability(seq+1, stateNum - 2) = 1;
%         end
% 
%         unclip_time(seq+1, stateNum - 2) = tEnd;
%         clip_time(seq+1, stateNum - 2) = tEnd;
% 
%         unclip_error(seq+1, stateNum - 2) = uncl_e_LS;
%         clip_error(seq+1, stateNum - 2) = cl_e_LS;
% 
%         save([save_directory, 'unclip_time.mat'], 'unclip_time');
%         save([save_directory, 'unclip_error.mat'], 'unclip_error');
%         save([save_directory, 'unclip_stability.mat'], 'unclip_stability');
% 
%         save([save_directory, 'clip_time.mat'], 'clip_time');
%         save([save_directory, 'clip_error.mat'], 'clip_error');
%         save([save_directory, 'clip_stability.mat'], 'clip_stability');
% 
%         fprintf('  LS  Max eigenvalue is : %.4f \n', maxeval_LS);
%         fprintf('  LS  Reconstruction error : %.5f \n', cl_e_LS);
% 
% 
% 
%         %% Compute SOC (stable) [A, B] solution
%         fprintf('Computing stable [A, B] solution using SOC ... \n');
% 
%         timeSOC = tic;
%         [A_SOC, ~] = learnSOCmodel(X_,Y_, options);
%         tEnd = toc(timeSOC);
% 
%         e_SOC = norm(Y_ - A_SOC*X_, 'fro')^2/2;
%         maxeval_SOC = max(abs(eig(A_SOC)));
% 
%         SOC_time(seq+1, stateNum - 2) = tEnd;
%         SOC_error(seq+1, stateNum - 2) = e_SOC;
% 
%         if(maxeval_SOC <= 1)
%             SOC_stability(seq+1, stateNum - 2) = 1;
%         end
% 
%         save([save_directory, 'SOC_time.mat'], 'SOC_time');
%         save([save_directory, 'SOC_error.mat'], 'SOC_error');
%         save([save_directory, 'SOC_stability.mat'], 'SOC_stability');
% 
%         fprintf(' SOC   Max eigenvalue is : %.4f \n', maxeval_SOC);
%         fprintf(' SOC   Reconstruction error : %.5f \n', e_SOC);    
% 
%         %% Compute WLS (stable) [A, B] solution
%         % fprintf('Computing stable A solution using WLS ... \n');
%         % 
%         % Pnew = [X_(:,1), Y_];
%         % [U_wls,S_wls,V_wls] = svd(Pnew,0);
%         % 
%         % n = stateNum;
%         % V_wls = V_wls(:,1:n);
%         % S_wls = S_wls(1:n,1:n);
%         % U_wls = U_wls(:,1:n);
%         % 
%         % timeWLS = tic;
%         % [A_WLS, ~, ~, ~] = learnWLSmodel(V_wls,S_wls,1,0);
%         % tEnd = toc(timeWLS);
%         % 
%         % e_WLS = norm(S_wls*V_wls(2:end,:)' - A_WLS * S_wls*V_wls(1:end-1,:)', 'fro')^2/2;
%         % maxeval_WLS = max(abs(eig(A_WLS)));
%         % if maxeval_WLS <= 1 
%         %     WLS_stability(seq+1, stateNum - 2) = 1;
%         % end
%         % WLS_error(seq+1, stateNum - 2) = e_WLS;
%         % WLS_time(seq+1, stateNum - 2) = tEnd;
%         % 
%         % save([save_directory, 'WLS_time.mat'], 'WLS_time');
%         % save([save_directory, 'WLS_error.mat'], 'WLS_error');
%         % save([save_directory, 'WLS_stability.mat'], 'WLS_stability');
%         % 
%         % fprintf('    Max eigenvalue is : %.4f \n', max(abs(eig(A_WLS)) ));
%         % fprintf('    Reconstruction error : %.5f \n', e_WLS);    
%   % % clearvars -except system
%   % 
%   %       %% Compute CG (stable) [A, B] solution
%         fprintf('Computing stable A using CG ... \n');
% 
%         timeCG = tic;
%         [A_CG, ~, ~, ~] = learnCGModel(X_, Y_, 1, 0);
%         tEnd = toc(timeCG);
% 
%         e_CG = norm(Y_ - A_CG*X_, 'fro')^2/2;
%         maxeval_CG = max(abs(eig(A_CG)));
%         if maxeval_CG <= 1 
%             CG_stability(seq+1, stateNum - 2) = 1;
%         end
%         CG_error(seq+1, stateNum - 2) = e_CG;
%         CG_time(seq+1, stateNum - 2) = tEnd;
% 
%         save([save_directory, 'CG_time.mat'], 'CG_time');
%         save([save_directory, 'CG_error.mat'], 'CG_error');
%         save([save_directory, 'CG_stability.mat'], 'CG_stability');
% 
%         fprintf('    Max eigenvalue is : %.4f \n', maxeval_CG)
%         fprintf('    Reconstruction error : %.5f \n', e_CG);    
% 
%     end
% end
