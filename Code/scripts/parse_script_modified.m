clear all; clc; close all;
%%
%%
clear rosbag_wrapper;
clear ros.Bag;
%%
bagpath = '/home/deepak/Desktop/PaperSubmissions/IROS2018/Code/';
folder = 'AllBAGFILES';
num_sub = 3;
global bagfile start_time;
load('trial_order_testing_8.mat');
%%
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

%%

% filename = 'H3PH1REHAT10.bag';
% bagfile = ros.Bag.load(filename);
% bagfile.info();
% get_start_time();
% user_vel = get_user_vel();
%%
pause on;
fnames = dir(strcat(bagpath, folder));
numfids = length(fnames);
count = 0;
frac_active = [];

uv_all = cell(numfids-3, 1);
alpha_all = cell(numfids-3, 1);
gv_all = cell(numfids-3, 1);
gp_all = cell(numfids-3, 1);
ms_all = cell(numfids-3, 1);
tt_all = cell(numfids-3, 1);
ig_all = cell(numfids-3, 1);
pos_all = cell(numfids-3, 1);
rot_all = cell(numfids-3, 1);
interface_all = cell(numfids-3, 1);
task_all = cell(numfids-3, 1);

ph0count = 0;
%%
for i=3:numfids-3
    filename = fnames(i).name;
    temp = filename; filename(end-3:end) = []; %remove .bag extension
    
    subId = str2num(filename(2));
    if length(filename) == 12
        trialId = str2double(filename(end-1:end));
    else
        trialId = str2double(filename(end));
    end
%     disp(filename);
    if strcmp(filename(3:5), 'PH0')
        ph0count = ph0count + 1;
        continue
    elseif strcmp(filename(3:5), 'PH1')
        ph = 1;
        ph_trial_list = ph1_trial_mat(:,:,subId-1);
    elseif strcmp(filename(3:5), 'PH2')
        ph = 2;
        ph_trial_list = ph2_trial_mat(:,:,subId-1);
    end
    intended_goal = ph_trial_list{trialId, 4};
    
%     if strcmp(filename(8:9), 'HA')
    disp(temp);
    bagfile = ros.Bag.load(temp);
    bagfile.info();
    get_start_time();
    
    %trial_time:
    total_time = bagfile.time_end - start_time;
    
    %get user_vel
    user_vel_ts = get_user_vel();
    
    %get mode switches
    ms = get_mode_switches();
    
    %get alpha values and upsample
    alpha_ts = get_alpha();
    alpha_upsampled = interp1(alpha_ts(:, 2), alpha_ts(:, 1), user_vel_ts(end, :)', 'linear', 'extrap');
    
    %get goal velocities and upsample
    gv_ts = get_goal_velocities(intended_goal);
    gv_upsampled = zeros(8, size(user_vel_ts, 2));
    for j = 1:size(gv_ts, 1) - 1
        gv_upsampled(j, :) = interp1(gv_ts(end, :), gv_ts(j, :), user_vel_ts(end, :), 'linear', 'extrap');
    end
    
    %get goal probabilities. 
    gp_ts = get_goal_probabilities();
    gp_upsampled = zeros(size(gp_ts, 1) - 1, size(user_vel_ts, 2));
    for j=1:size(gp_ts, 1) - 1
        gp_upsampled(j, :) = interp1(gp_ts(end, :), gp_ts(j, :), user_vel_ts(end, :), 'linear', 'extrap');
    end
    
    %get eef position trajectory.
    
    tree = ros.TFTree(bagfile);
    tree.allFrames();
    source_frame = 'mico_link_base';
    target_frame = 'mico_link_hand';
%     deltat = 0.01;
%     times = (tree.time_begin:deltat:tree.time_end-1)';
%     t = times-start_time;
%     first_t = find(t < 0); first_t = first_t(end) + 1;
%     times(1:first_t) = [];
%     times = user_vel_ts(end, :)' + start_time; %query tf at same time stamps as user_vel
    % create timestamps for tf. Kinda funny because of tf api issues in
    % matlab
    tftimes = user_vel_ts(end, :)' + start_time;
    tftimes(tftimes > (tree.time_end - 1)) = tree.time_end - 1;
    tftimes(tftimes < (tree.time_begin + 1)) = tree.time_begin + 1; 
    mico_xyz = tree.lookup(source_frame, target_frame, tftimes);
    positions = zeros(3, length(mico_xyz));
    orientations = zeros(4, length(mico_xyz));
    for ii=1:length(mico_xyz)
        positions(:, ii) = mico_xyz(ii).translation;
        orientations(:, ii) = mico_xyz(ii).rotation;                      
    end

    %%
    
    uv_all{i-2} = user_vel_ts;
    alpha_all{i-2} = alpha_upsampled;
    gv_all{i-2} = gv_upsampled;
    ms_all{i-2}= ms;
    ig_all{i-2} = intended_goal;
    gp_all{i-2} = gp_upsampled;
    tt_all{i-2} = total_time;
    pos_all{i-2} = positions;
    rot_all{i-2} = orientations;
    
    if strcmp(filename(8:9), 'J2')
        interface_all{i-2} = 1;
    else
        interface_all{i-2} = 2;
    end
    if strcmp(filename(6:7), 'RE')
        task_all{i-2} = 1;
    else
        task_all{i-2} = 2; 
    end
    
    
        %user_vel is happening around 100Hz whereas alpha is happening
        %around 20Hz. Choice between upsampling alpha or downsampling
        %user_vel
        
%         figure; hold on; grid on;
%         for j=1:6
%             plot(user_vel_ts(end, :), gv_upsampled(j, :)); %b.r.y.purple.green.lightblue
% %             plot(user_vel_ts(end, :), user_vel_ts(j, :)', 'LineWidth', 2.0);
%         end
%         plot(user_vel_ts(end, :), user_vel_ts(1:end-1, :)', 'LineWidth', 2.0);
%         plot(user_vel_ts(end, :), sum(abs(user_vel_ts(1:end-1, :)) > 0), 'LineWidth', 2.0);
        
%         plot(user_vel_ts(end, :), alpha_upsampled, 'LineWidth', 3.0);
        
        
%     end
    

%     
end

%%
ms_all(cellfun(@isempty, gv_all)) = [];
tt_all(cellfun(@isempty, gv_all)) = [];
uv_all(cellfun(@isempty, uv_all)) = [];
alpha_all(cellfun(@isempty, alpha_all)) = [];
ig_all(cellfun(@isempty, ig_all)) = [];
gv_all(cellfun(@isempty, gv_all)) = [];
gp_all(cellfun(@isempty, gp_all)) = [];
interface_all(cellfun(@isempty, interface_all)) = [];
task_all(cellfun(@isempty, task_all)) = [];
pos_all(cellfun(@isempty, pos_all)) = [];
rot_all(cellfun(@isempty, rot_all)) = [];

%%
% 
% for i=1:num_sub
%     user_ID = subList{i};
%     fnames = dir(strcat(bagpath, user_ID));
%     numfids =length(fnames);
%     for j=3:numfids-1
%         filename = fnames(j).name;
%         disp(filename);
%         if strcmp(filename(3:5), 'PH0')
%             continue
%         else
%             
%             bagfile = ros.Bag.load(filename);
%             bagfile.info()
%             get_start_time();
%             user_vel = get_user_vel();
% %             get_start_time();
% %             tree = ros.TFTree(bagfile);
% %             tree.allFrames();
% %             source_frame = 'mico_link_base';
% %             target_frame = 'mico_link_hand';
% %             deltat = 0.02;
% %             times = (tree.time_begin:deltat:tree.time_end-1)';
% %             t = times-start_time;
% %             first_t = find(t < 0); first_t = first_t(end) + 1;
% %             t(1:first_t) = [];
% %             mico_xyz = tree.lookup(source_frame, target_frame, times);
% %             positions = zeros(3, length(mico_xyz));
% %             orientations = zeros(4, length(mico_xyz));
% %             for ii=1:length(mico_xyz)
% %                 positions(:, ii) = mico_xyz(ii).translation;
% %                 orientations(:, ii) = mico_xyz(ii).rotation;                      
% %             end
% %             
% %             close all;figure;
% %             scatter3(positions(1, :), positions(2,:), positions(3,:))
% %             hold on; grid on;
% %             scatter3(positions(1, 1), positions(2,1), positions(3,1), 120, 'r', 'filled');
% %             scatter3(positions(1, end), positions(2,end), positions(3,end), 120, 'g', 'filled');
% %             scatter3(pouring_goal_pos(1,:), pouring_goal_pos(2,:), pouring_goal_pos(3,:), 180, 'k', 'filled'); grid on; hold on;
% %             % xlabel('x'); ylabel('y'); zlabel('z');
% %             xlabel('X'); ylabel('Y'); zlabel('Z');
% %             view([-142,32]);
% %             t = times - t0;
% %             alpha = get_alpha();
% %             assis_req = get_assis_req();
% %             cm = get_current_mode();
% %             ms = get_mode_switches();
% %             disp(length(cm)); disp(length(cm));
% %             gp = get_goal_probabilities();
% %             gv = get_goal_velocities();
%             
%             
%         end
%         
%     end
%     
%     
% end

%HELPER FUNCTIONS
function gv = get_goal_velocities(intended_goal)
    global start_time;
    topic = '/goal_velocities';
    [msgs, meta] = get_bag_data(topic);
    gv = zeros(length(msgs{10}.goal_vels(1).data)  + 1, length(msgs));
    for i=1:length(msgs)
        if ~isempty(msgs{i}.goal_vels(intended_goal).data)
            gv(1:end-1, i) = msgs{i}.goal_vels(intended_goal).data;
            gv(end, i) = meta{i}.time.time - start_time;
        end
    end
    gv = trim_data(gv, true);
    gv = round(gv, 4);

end
%%
function gp = get_goal_probabilities()
    global start_time;
    topic = '/goal_probabilities';
    [msgs, meta] = get_bag_data(topic);
    gp = zeros(length(msgs{1}.data) + 1, length(msgs));
    
    for i=1:length(msgs)
        gp(1:end-1, i) = msgs{i}.data;
        gp(end, i) = meta{i}.time.time - start_time;
    end
    gp = trim_data(gp, true);
    gp = round(gp, 4);
%     disp(gp);
    
end
function ms = get_mode_switches()
    global start_time;
    topic = '/mode_switches';
    [msgs, meta] = get_bag_data(topic);
    ms = zeros(length(meta), 1);
    for i=1:length(meta)
        ms(i) = meta{i}.time.time -start_time;
    end
    ms(ms < 0) = [];
end
function ms = get_current_mode()
    global start_time;
    topic = '/mi/current_mode';
    [msgs, meta] = get_bag_data(topic);
    ms = zeros(length(meta), 1);
    for i=1:length(meta)
        ms(i) = meta{i}.time.time -start_time;
    end
    repeated_index = find(diff(ms) < 0.001) + 1;
    ms(repeated_index) = [];
    ms(ms < 0) = [];
end
function assis_req = get_assis_req()
    global start_time;
    topic = '/assistance_requested';
    [msgs, meta] = get_bag_data(topic);
    assis_req = zeros(length(msgs), 1);
    for i=1:length(msgs)
        assis_req(i) = meta{i}.time.time - start_time;
    end
    
end
function user_vel = get_user_vel()
    global start_time;
    topic = '/user_vel';
    [msgs, meta] = get_bag_data(topic);
    user_vel = zeros(length(msgs{1}.data(1:6)) + 1, length(msgs));
    for i=1:length(msgs)
        user_vel(1:end-1, i) = msgs{i}.data(1:6);
        user_vel(end, i) = meta{i}.time.time - start_time;
    end
    user_vel = trim_data(user_vel, true);
%     user_vel = round(gp, 4);
    
        
end
function alpha = get_alpha()
    global start_time;
    topic = '/alpha';
    [msgs, meta] = get_bag_data(topic);
    alpha = zeros(length(msgs), 1);
    alpha_t = zeros(length(msgs), 1);
    for i=1:length(msgs)
        alpha(i, 1) = msgs{i};
        alpha_t(i, 1) = meta{i}.time.time - start_time;
    end
    alpha = [alpha, alpha_t];
    alpha = trim_data(alpha, false);
    
end
function  get_start_time()
    global start_time;
    topic = '/start_end';
    [~, meta] = get_bag_data(topic);
    start_time = meta{1}.time.time;
end

function [msgs, meta] = get_bag_data(topic)
    global bagfile;
    [msgs, meta] = bagfile.readAll({topic});
end

function [td] = trim_data(d, isrow)
    if isrow
        ts = d(end, :);
    else
        ts = d(:, end);
    end
    first_t = find(ts < 0);
    if ~isempty(first_t)
        first_t = first_t(end);
        if isrow()
            d(:, 1:first_t) = [];
        else
            d(1:first_t, :) = [];
        end
    end
    td = d;
end