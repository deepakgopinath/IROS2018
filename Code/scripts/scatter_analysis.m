
clear all; clc; close all;
%%
load('j_uv_to_gp.mat'); load('new_data_file.mat'); load('interface_task_TE_new.mat');
load('j_uv_to_alpha.mat');
%%

pred = j_uv_to_alpha(j_uv_to_alpha~=0 &j_uv_to_alpha~=-999);
resp = cell2mat(tt_all(j_uv_to_alpha~=0 &j_uv_to_alpha~=-999));
scatter(pred, resp, 'filled', 'b');

%%
hold on;
pred = j_uv_to_gp(j_uv_to_gp~=0 &j_uv_to_gp~=-999);
resp = cell2mat(tt_all(j_uv_to_gp~=0 &j_uv_to_gp~=-999));
scatter(pred, resp, 'filled', 'g');

%%
hold on; grid on;
pred = [j2reTElist(j2reTElist ~= 0 & j2reTElist ~= -999); j2poTElist(j2poTElist ~= 0 & j2poTElist ~= -999)];
resp = [cell2mat(tt_all(j2reTElist ~= 0 & j2reTElist ~= -999)); cell2mat(tt_all(j2poTElist ~= 0 & j2poTElist ~= -999))];
scatter(pred, resp, 'filled', 'r');

%% scatter between TE_hr and TErh
ind = find(j_uv_to_alpha~=0 &j_uv_to_alpha~=-999);
x = j_uv_to_alpha;
y = j2reTElist + j2poTElist;
for i=8:8
    sub_ind = intersect(ind, init_sub_markers(i):sub_markers(i));
    scatter(x(sub_ind), y(sub_ind), 'filled'); hold on; grid on;
end
xlabel('TE_hr'); ylabel('TE_rh');

% x = j_uv_to_alpha(j_uv_to_alpha~=0 &j_uv_to_alpha~=-999);
% y = [j2reTElist(j2reTElist ~= 0 & j2reTElist ~= -999); j2poTElist(j2poTElist ~= 0 & j2poTElist ~= -999)];

