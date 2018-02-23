clear all; clc; close all;
%%
javaaddpath('../../InfoDynamics/infodynamics.jar');
load('new_data_file.mat');
num_trials = size(alpha_all, 1);
num_sub = 8; %foir the time being just look at one subject
trials_per_sub = 32*ones(num_sub,1); trials_per_sub(5) = 31;

aisCalc=javaObject('infodynamics.measures.continuous.kraskov.ActiveInfoStorageCalculatorMultiVariateKraskov');
teCalc=javaObject('infodynamics.measures.continuous.kraskov.TransferEntropyCalculatorMultiVariateKraskov');
    
j_id = 1; ha_id = 2; re_id = 1; po_id = 2;
reaching_goal_pos = [   0.328, -0.516, 0.1;
                        -0.116, -0.579, 0.132;
                        0.124, -0.531, 0.442;
                        -0.346, -0.389, 0.220;
                        -0.376, -0.180, 0.102
                        ]';
pouring_goal_pos = [    0.3314, -0.206, 0.1;
                        0.306, -0.550, 0.256;
                        -0.05, -0.627, 0.225;
                        -0.370, -0.55504, 0.3955
                        ]';
                    
sub_markers = cumsum(trials_per_sub); init_sub_markers = sub_markers - 31; init_sub_markers(5) = 129;
global_TE_list = zeros(length(alpha_all), 1);
pval_list = zeros(length(alpha_all), 1);
%%
for jj=1:num_sub
    for i=init_sub_markers(jj):sub_markers(jj) %all trials for a specific subjetc. 
        if interface_all{i} == ha_id  %if it is headarray or pouring skip. only interested in joystick and reaching
            continue
        end
        if ms_all{i} == -999 %if failed trial skip as well. 
            global_TE_list(i) = -999;
            continue;
        end
        alpha = alpha_all{i}; %alpha = (alpha > 0)*1;
        gv = gv_all{i}; gv =  gv'; gv(:, end-1:end) = []; %remove time index row. 
        gv = repmat(alpha, 1, size(gv, 2)).*gv; % the portion of robot autonomy that the human sees. 

        gp = gp_all{i}; gp = gp(ig_all{i}, :)'; %goal probability associated with intended goal. 
        uv = uv_all{i}; uv = uv'; uv = uv(:, 1:end-1); %user velocity from interface
        ts = uv_all{i}(end, :)'; %time stamps
        ms = ms_all{i}; %mode switch timings. 
        total_time = tt_all{i};
        
        %robot to human
        sourceMVArray = gv;
        destMVArray = uv;
        sourceDim = size(sourceMVArray, 2);  
        destDim = size(destMVArray, 2);
        knn = 4; %neighbors for Kraskov estimation. fixed according to KSG original paper. 
        %human to robot.
    %     sourceMVArray = uv;
    %     destMVArray = gp;
    %     sourceDim = size(sourceMVArray, 2);  
    %     destDim = size(destMVArray, 2);
    %     knn = 4;
    
        delay = 1;
%         teCalc.initialise(sourceDim, destDim, str2num(kUsedS), str2num(kTauUsedS), str2num(kUsedD), str2num(kTauUsedD), delay);
        teCalc.initialise(sourceDim, destDim, 1,1,1,1, delay);
        teCalc.setProperty('ALG_NUM', '1');
        teCalc.setProperty('NORMALISE', 'true');   
        teCalc.setProperty('NOISE_LEVEL_TO_ADD', '0.0000001');
        teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
        teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
        global_TE = teCalc.computeAverageLocalOfObservations();
        fprintf('The TE is %f\n', global_TE);
        global_TE_list(i) = global_TE;
        local_TE = teCalc.computeLocalOfPreviousObservations();
        
        repeats = 100;
        nullDist = teCalc.computeSignificance(repeats);
        empCalc=javaObject('infodynamics.utils.EmpiricalMeasurementDistribution', nullDist.distribution, global_TE);
        fprintf('The p-value is %f for trial %d\n', empCalc.pValue, i);
%         distribution = javaMatrixToOctave(nullDist.distribution);
%         pvalue = sum(distribution >= nullDist.actualValue)/length(distribution);
        pval_list(i) = empCalc.pValue;
    %% plot figure. 
%         figure; hold on; grid on;
%         plot(ts, local_TE, 'g', 'LineWidth', 2.0); 
%         plot(ts, sourceMVArray, 'r', 'LineWidth', 1.5);  plot(ts, destMVArray, 'b', 'LineWidth', 1.5);
%         plot(ts, alpha_all{i}, 'm', 'LineWidth', 1.5);
%         scatter(ms, 0.5*ones(length(ms), 1), 'filled');
%         
%         close all;
        
    end
end
%% scatter plottin
% load('j2reTE.mat'); load('j2poTE.mat'); load('data_file.mat');
% figure; hold on;
% 
% scatter(j2reTElist(j2reTElist ~= 0), cell2mat(tt_all(j2reTElist ~= 0)), 'r', 'filled');
% scatter(j2poTElist(j2poTElist ~= 0), cell2mat(tt_all(j2poTElist ~= 0)), 'b', 'filled');
% 
% grid on;

%%
load('interface_task_TE_new.mat'); load('new_data_file.mat');
% 
% %%
% figure;
hold on; grid on;
scatter(j2reTElist(j2reTElist ~= 0 & j2reTElist ~= -999), cell2mat(tt_all(j2reTElist ~= 0 & j2reTElist ~= -999)), 'r', 'filled');
% scatter(j2poTElist(j2poTElist ~= 0 & j2poTElist ~= -999), cell2mat(tt_all(j2poTElist ~= 0 & j2poTElist ~= -999)), 'b', 'filled');
%%
figure; hold on; grid on;
scatter(hareTElist(hareTElist ~= 0 & hareTElist ~= -999), cell2mat(tt_all(hareTElist ~= 0 & hareTElist ~= -999)), 'r', 'filled', 'd');
scatter(hapoTElist(hapoTElist ~= 0 & hapoTElist ~= -999), cell2mat(tt_all(hapoTElist ~= 0 & hapoTElist ~= -999)), 'b', 'filled', 'd');
%%
% %% Linear Regression for each interface-task type. 
% pred = j2reTElist(j2reTElist ~= 0);
% resp = cell2mat(tt_all(j2reTElist ~= 0));
% [fitobject, gof] = fit(pred, resp, 'poly1', 'Robust', 'LAR');
% %%
pred = j2reTElist(j2reTElist ~= 0 & j2reTElist ~= -999);
resp = cell2mat(tt_all(j2reTElist ~= 0 & j2reTElist ~= -999));
[fitobject, gof] = fit(pred, resp, 'poly1', 'Robust', 'LAR');
% %%
% pred = j2poTElist(j2poTElist ~= 0 & j2poTElist ~= -999);
% resp = cell2mat(tt_all(j2poTElist ~= 0 & j2poTElist ~= -999));
% [fitobject, gof] = fit(pred, resp, 'poly1', 'Robust', 'LAR');
% %%
% pred = hareTElist(hareTElist ~= 0 & hareTElist ~= -999);
% resp = cell2mat(tt_all(hareTElist ~= 0 & hareTElist ~= -999));
% [fitobject, gof] = fit(pred, resp, 'poly1', 'Robust', 'LAR');
% %%
% pred = hapoTElist(hapoTElist ~= 0 & hapoTElist ~= -999);
% resp = cell2mat(tt_all(hapoTElist ~= 0 & hapoTElist ~= -999));
% [fitobject, gof] = fit(pred, resp, 'poly1', 'Robust', 'LAR');

