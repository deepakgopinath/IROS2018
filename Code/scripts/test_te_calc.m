clear all; clc; close all;
%%

javaaddpath('/home/deepak/Desktop/Research/PaperSubmissions/IROS2018/InfoDynamics/infodynamics.jar');
load('data_file.mat');
num_trials = size(alpha_all, 1);
num_sub = 8;
trials_per_sub = 32*ones(num_sub,1); trials_per_sub(5) = 31;
teCalc=javaObject('infodynamics.measures.continuous.kraskov.TransferEntropyCalculatorMultiVariateKraskov');
total_TE = zeros(num_trials, 1);
total_TE_onlyalpha = zeros(num_trials, 1);
sub_markers = cumsum(trials_per_sub); init_sub_markers = sub_markers - 31; init_sub_markers(5) = 129;
index = 1;
fprintf('Subject ID is %d\n', index);
subjectwise_total_TEs = cell(num_sub, 1);
j_id = 1; ha_id = 2; re_id = 1; po_id = 2;
%%
for i=1:num_trials
    if i > sub_markers(index)
        mean_TE = mean(total_TE(init_sub_markers(index): sub_markers(index)));
        subjectwise_total_TEs{index} = total_TE(init_sub_markers(index): sub_markers(index));
        mean_TE = sum(total_TE(init_sub_markers(index):sub_markers(index)))/sum(total_TE(init_sub_markers(index):sub_markers(index)) ~= 0);
        fprintf('The mean TE for subject %d is %f\n', index, mean_TE);
        index = index + 1;
        fprintf('#############################\n');
        fprintf('Subject ID is %d\n', index)
    end
    %prepare source and destination data. 
    if interface_all{i} == ha_id %if it is headarray
        continue
    end
    
    alpha = alpha_all{i}; alpha = (alpha > 0)*1;
    gv = gv_all{i}; gv =  gv'; gv(:, end-1:end) = [];
    gv = repmat(alpha, 1, size(gv, 2)).*gv;
    uv = uv_all{i}; uv = uv'; uv = uv(:, 1:end-1);
    ts = uv_all{i}(end, :)';
    ms = ms_all{i};
    
    sourceMVArray = gv;
    destMVArray = uv ;
    
    sourceDim = size(sourceMVArray, 2);  
    destDim = size(destMVArray, 2);
    
    teCalc.initialise(1, sourceDim,destDim); % Use history length 1 (Schreiber k=1)
    teCalc.setProperty('k', '4'); % Use Kraskov parameter K=4 for 4 nearest points
    teCalc.setObservations(octaveToJavaDoubleMatrix(sourceMVArray), octaveToJavaDoubleMatrix(destMVArray));
    total_TE(i) = teCalc.computeAverageLocalOfObservations();
    local_TE = teCalc.computeLocalOfPreviousObservations();
    figure;
    plot(ts, local_TE, 'g', 'LineWidth', 2.0); hold on;
    plot(ts, sourceMVArray, 'r', 'LineWidth', 1.5);  plot(ts, destMVArray, 'b', 'LineWidth', 1.5);
    plot(ts, alpha_all{i}, 'm', 'LineWidth', 1.5);
    scatter(ms, 0.5*ones(length(ms), 1), 'filled');
    fprintf('The transfer entropy from robot alpha to presence of user vel is %f\n', total_TE(i));
    repeats = 500;
    nullDist = teCalc.computeSignificance(repeats);
    distribution = javaMatrixToOctave(nullDist.distribution);
    pvalue = sum(distribution >= nullDist.actualValue)/length(distribution);
    
    total_TE_onlyalpha(i) = mean(local_TE(alpha == 1));
    
%     %initialize TE calc for every trial. (Later we can pool together each
%     %user's signal together. 
%     teCalc.initialise();
%     teCalc.addObservations(sourceArray, destArray);
%     
%     %compute_total_TE
%     total_TE(i) = teCalc.computeAverageLocalOfObservations();
%     local_TE = teCalc.computeLocalFromPreviousObservations(sourceArray, destArray);
%     
%     plot all signals. 
%     figure;
%     plot(ts, local_TE, 'g', 'LineWidth', 2.0); hold on;
%     plot(ts, sourceMVArray, 'r', 'LineWidth', 1.5);  plot(ts, destMVArray, 'b', 'LineWidth', 1.5);
%     plot(ts, alpha_all{i}, 'm', 'LineWidth', 1.5);
%     scatter(ms, 0.5*ones(length(ms), 1), 'filled');
    fprintf('The transfer entropy from robot alpha to presence of user vel is %f\n', total_TE(i));
    fprintf('The mean transfer entropy from robot alpha to presence of user vel for nonzero alpha is %f\n', mean(local_TE(alpha == 1)));
    
%     close all;
end
mean_TE = mean(total_TE(init_sub_markers(index): sub_markers(index)));
subjectwise_total_TEs{index} = total_TE(init_sub_markers(index): sub_markers(index));
fprintf('The mean TE for subject %d is %f\n', index, mean_TE);

%% clean up zeros 
for i=1:num_sub
    subjectwise_total_TEs{i} =  subjectwise_total_TEs{i}(subjectwise_total_TEs{i} ~= 0);
end


%%
subjectwise_total_TE_only_alpha = cell(num_sub, 1);
for i=1:num_sub
     subjectwise_total_TE_only_alpha{i} = total_TE_onlyalpha(init_sub_markers(i): sub_markers(i));
end

