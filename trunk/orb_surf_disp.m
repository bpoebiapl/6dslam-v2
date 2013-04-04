run surf_data
run orb_data

AICK_surf_30_inf 	= originalAICKsurf_0_AICK_mat_pos;
AICK_surf_30_inf_time 	= originalAICKsurf_0_AICK_avg_time;
AICK_surf_5_200  	= originalAICKsurf_1_AICK_mat_pos;
AICK_surf_5_200_time 	= originalAICKsurf_1_AICK_avg_time;

AICK_orb_30_inf 	= originalAICKorb_0_AICK_mat_pos;
AICK_orb_30_inf_time 	= originalAICKorb_0_AICK_avg_time;
AICK_orb_10_300  	= originalAICKorb_1_AICK_mat_pos;
AICK_orb_10_300_time 	= originalAICKorb_1_AICK_avg_time;

disp('Running matlab script');
%figure(1)
%clf
%hold on
%part = 1:75;
%axis([0 thresholds(part(end)) 0 1.01])
%step = 1;%:5:30;
%plot(thresholds(part),AICK_surf_30_inf(step,(part)),'g')
%plot(thresholds(part),AICK_surf_5_200(step,(part)),'g-x')
%plot(thresholds(part),AICK_orb_30_inf(step,(part)),'r')
%plot(thresholds(part),AICK_orb_10_300(step,(part)),'r-x')

%plot(thresholds(part),bowAICKsurf_bl_0_15_0_BowAICKv2_mat_pos(step,(part)),'b-x')
%plot(thresholds(part),bowAICKsurf_bl_0_2_0_BowAICKv2_mat_pos(step,(part)),'b-o')
%plot(thresholds(part),bowAICKsurf_bl_0_25_0_BowAICKv2_mat_pos(step,(part)),'m')
%plot(thresholds(part),bowAICKsurf_bl_0_3_0_BowAICKv2_mat_pos(step,(part)),'m-x')
plot(thresholds(part),bowAICKsurf_bl_0_35_0_BowAICKv2_mat_pos(step,(part)),'m-o')




%legend('AICK surf','AICK surf fast','AICK orb','AICK orb fast')



% figure(2)
% clf
% hold on
% plot(originalAICKsurf_1_AICK_mat_pos(:,2*15),'g')
% plot(bowAICKsurf1000_bl_02_3_BowAICKv2_mat_pos(:,2*15),'g-x')

figure(3)
clf
hold on
plot(log2(AICK_surf_30_inf_time),'g')
plot(log2(AICK_surf_5_200_time),'g-x')
plot(log2(AICK_orb_30_inf_time),'r')
plot(log2(AICK_orb_10_300_time),'r-x')

plot(log2(bowAICKsurf_bl_0_15_0_BowAICKv2),'b-x')
plot(log2(bowAICKsurf_bl_0_2_0_BowAICKv2),'b-o')
plot(log2(bowAICKsurf_bl_0_25_0_BowAICKv2),'m')
plot(log2(bowAICKsurf_bl_0_3_0_BowAICKv2),'m-x')
plot(log2(bowAICKsurf_bl_0_35_0_BowAICKv2),'m-o')

%plot(log2(originalAICKsurf_3_AICK_avg_time),'g')
%plot(log2(bowAICKsurf1000_bl_02_2_BowAICKv2_avg_time),'g-x')
%plot(log2(originalAICKorb_0_AICK_avg_time),'r')
%plot(log2(originalAICKorb_1_AICK_avg_time),'r-x')
