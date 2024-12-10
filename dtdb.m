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
save_directory = 'results_dtdb/';

mkdir(save_directory);


% Whether resume training: 0 for new training, 1 for resuming training
status = 0;

addpath(algorithms_path);
options.graphic = 0;
options.posdef = 10e-12;
options.maxiter = 200000;

% settings;
nStates = 17; % Number of system states
nControl = 7; % Number of system inputs

if(status == 0)
    LS_error = zeros(254, 27);
    SC_error = zeros(254, 27);
    WLS_error = zeros(254, 27);
    CG_error = zeros(254, 27);
    SOC_error = zeros(254, 27);
    
    LS_time = zeros(254, 27);
    SC_time = zeros(254, 27);
    WLS_time = zeros(254, 27);
    CG_time = zeros(254, 27);
    SOC_time = zeros(254, 27);
    
else
    CG_error = load('results_svd/dtdb/CG_error.mat').CG_error;
    CG_time = load('results_svd/dtdb/CG_time.mat').CG_time;
    
    SC_error = load('results_svd/dtdb/SC_error.mat').SC_error;
    SC_time = load('results_svd/dtdb/SC_time.mat').SC_time;
    
    LS_error = load('results_svd/dtdb/LS_error.mat').LS_error;
    LS_time = load('results_svd/dtdb/LS_time.mat').LS_time;
    
    WLS_error = load('results_svd/dtdb/WLS_error.mat').WLS_error;
    WLS_time = load('results_svd/dtdb/WLS_time.mat').WLS_time;
    
    SOC_error = load('results_svd/dtdb/SOC_error.mat').SOC_error;
    SOC_time = load('results_svd/dtdb/SOC_time.mat').SOC_time;
end

dataRoot = 'prepare_data/dtdb/prepared/';
for seq = 0:285
    disp(['current seq ', num2str(seq)]);
    loc = [dataRoot, 'dtdb_', num2str(seq + 1), '.mat'];
    data = double(load(loc).seq.') / 255.0;
    [total_state, length] = size(data);
    if(length > 300)
        data = data(:, 1:300);
    end

    [data_U, data_S, data_V] = svd(data);


    for stateNum = 3:30
        SOCspace_S = data_S(1:stateNum, :);
        SOCspace_data = SOCspace_S * data_V.';
        disp(['dimension ', num2str(stateNum)])
        X_ = SOCspace_data(:, 1:end - 1);
        Y_ = SOCspace_data(:, 2:end);


        % %% Compute LS (unconstrained) [A, B] solution
        tStart = tic;
        tStart_SC = tic;
        LS = LDS(X_, Y_);
        tLS = toc(tStart);
        SC = eigenclip(LS, 1);
        tSC = toc(tStart_SC); 
        ls_error = mean(mean(abs(Y_ - LS * X_)));
        sc_error = mean(mean(abs(Y_ - SC * X_)));

        LS_time(seq+1, stateNum - 2) = tLS;
        SC_time(seq+1, stateNum - 2) = tSC;

        LS_error(seq+1, stateNum - 2) = ls_error;
        SC_error(seq+1, stateNum - 2) = sc_error;

        save([save_directory, 'LS_time.mat'], 'LS_time');
        save([save_directory, 'LS_error.mat'], 'LS_error');

        save([save_directory, 'SC_time.mat'], 'SC_time');
        save([save_directory, 'SC_error.mat'], 'SC_error');


        %% Compute SOC (stable) [A, B] solution

        timeSOC = tic;
        [SOC, ~] = learnSOCmodel(X_,Y_, options);
        tSOC = toc(timeSOC);

        soc_error = mean(mean(abs(Y_ - SOC*X_)));

        SOC_time(seq+1, stateNum - 2) = tSOC;
        SOC_error(seq+1, stateNum - 2) = soc_error;


        save([save_directory, 'SOC_time.mat'], 'SOC_time');
        save([save_directory, 'SOC_error.mat'], 'SOC_error');
    

        %% Compute WLS (stable) [A, B] solution
  
        Pnew = [X_(:,1), Y_];
        [U_wls,S_wls,V_wls] = svd(Pnew,0);

        n = stateNum;
        V_wls = V_wls(:,1:n);
        S_wls = S_wls(1:n,1:n);
        U_wls = U_wls(:,1:n);

        timeWLS = tic;
        [WLS, ~, ~, ~] = learnWLSmodel(V_wls,S_wls,1,0);
        tWLS = toc(timeWLS);
        % 
        wls_error = mean(mean(abs(S_wls*V_wls(2:end,:)' - A_WLS * S_wls*V_wls(1:end-1,:)')));

        WLS_error(seq+1, stateNum - 2) = error_wls;
        WLS_time(seq+1, stateNum - 2) = tWLS;
        % 
        save([save_directory, 'WLS_time.mat'], 'WLS_time');
        save([save_directory, 'WLS_error.mat'], 'WLS_error');
 
        %% Compute CG (stable) [A, B] solution

        timeCG = tic;
        [A, ~, ~, ~] = learnCGModel(X_, Y_, 1, 0);
        tCG = toc(timeCG);

        cg_error = mean(mean(abs(Y_ - A_CG*X_)));
        CG_error(seq+1, stateNum - 2) = cg_error;
        CG_time(seq+1, stateNum - 2) = tCG;

        save([save_directory, 'CG_time.mat'], 'CG_time');
        save([save_directory, 'CG_error.mat'], 'CG_error');

    end
end
