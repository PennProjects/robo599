%% load csv
raw_mat_nm_1 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/nomov/mat/2021-4-29_16_51_nomov_1.csv');
raw_mat_rh_1 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/right_hand/mat/2021-4-29_16_52_Rgthnd_1.csv');
raw_mat_rl_1 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/right_leg/mat/2021-4-29_16_55_rgtleg_1.csv');
raw_mat_ll_3 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/left_leg/mat/2021-4-29_17_3_lftleg_3.csv');


%%
%plot raw x vx y
% plot(datset.Var5(3:10,:), datset.Var6(3:10,:), 'o-');
figure();
datset  = raw_mat_nm_1;
for i = 1:size(datset,1)
    plot(datset.Var5(i), datset.Var6(i), 'o-')
    xlim([-20,5])
    ylim([-10,30])
    hold on
    drawnow
end

%%
figure();
datset  = raw_mat_rl_1;
for i = 1:size(datset,1)
    plot(datset.Var5(i), datset.Var6(i), 'o-')
    xlim([-20,5])
    ylim([-10,30])
    hold on
    drawnow
end

%%
subplot(1,2,1)
datset  = raw_mat_rl_1;
plot(datset.Var5, datset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(1,2,2)
datset  = raw_mat_ll_3
plot(datset.Var5, datset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])
%%