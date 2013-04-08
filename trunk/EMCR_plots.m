disp('generating EMCR plots')
run orb_data
run surf_data
run GICP_and_NDT_data

max_trid = 75;
step = 1;
threshold_id = 15;

high_performance = 5;
mid_performance = 15;
low_performance = 75;

close all

plotcol = [
    'r  ';
    'g  ';
    'b  ';
    'c  ';
    'm  ';
    'r-o';
    'g-o';
    'b-o';
    'c-o';
    'm-o';
    'r-x';
    'g-x';
    'b-x';
    'c-x';
    'm-x';
    'r  ';
    'g  ';
    'b  ';
    'c  ';
    'm  ';
    'r-x';
    'g-x';
    'b-x';
    'c-x';
    'm-x';
    'r-o';
    'g-o';
    'b-o';
    'c-o';
    'm-o';
];

aick_orb    = originalAICKorb_0_AICK_mat_pos;
%aick_time   = test0008Surf_3_AICK_avg_time;

aick_surf    = originalAICKorb_0_AICK_mat_pos;
%aick_time   = test0008Surf_3_AICK_avg_time;

ndt         = GICPandNDTroom1_0_NDTMatcher_mat_pos;
ndt_time    = GICPandNDTroom1_0_NDTMatcher_avg_time;

gicp        = GICPandNDTroom1_1_BasicGIcpMatcher_mat_pos;
gicp_time   = GICPandNDTroom1_1_BasicGIcpMatcher_avg_time;

%%

figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');
plot(thresholds(1:max_trid),aick_surf(step,(1:max_trid)),'r','LineWidth',2.0);
plot(thresholds(1:max_trid),aick_orb(step,(1:max_trid)),'g','LineWidth',2.0);
plot(thresholds(1:max_trid),ndt(step,(1:max_trid)),'b','LineWidth',2.0);
plot(thresholds(1:max_trid),gicp(step,(1:max_trid)),'c','LineWidth',2.0);

legend('AICK surf','AICK orb','NDT','GICP','Location','SouthEast')

figure()
axis([0 1 0 1.01])
hold on
title(['Translation error <' num2str(thresholds(threshold_id))])
xlabel('time between frames [s]');
ylabel('success ratio');

data = aick_surf(:,threshold_id);
plot((1:size(data,1))/30,data,'r','LineWidth',2.0);

data = aick_orb(:,threshold_id);
plot((1:size(data,1))/30,data,'g','LineWidth',2.0);

data = ndt(:,threshold_id);
plot((1:size(data,1))/30,data,'b','LineWidth',2.0);

data = gicp(:,threshold_id);
plot((1:size(data,1))/30,data,'c','LineWidth',2.0);

legend('AICK surf','AICK orb','NDT','GICP','Location','NorthEast')


%%
varyKeypointsDataOrb = [];
varyKeypointsDataOrb(1,:,:)=originalAICKorbKeyPoints_0_AICK_mat_pos; 
varyKeypointsDataOrb(2,:,:)=originalAICKorbKeyPoints_1_AICK_mat_pos; 
varyKeypointsDataOrb(3,:,:)=originalAICKorbKeyPoints_2_AICK_mat_pos; 
varyKeypointsDataOrb(4,:,:)=originalAICKorbKeyPoints_3_AICK_mat_pos; 
varyKeypointsDataOrb(5,:,:)=originalAICKorbKeyPoints_4_AICK_mat_pos; 
varyKeypointsDataOrb(6,:,:)=originalAICKorbKeyPoints_5_AICK_mat_pos;


varyKeypointsNameOrb = [
    'AICK orb max 500 keypoints'
    'AICK orb max 400 keypoints'
    'AICK orb max 350 keypoints'
    'AICK orb max 300 keypoints'
    'AICK orb max 200 keypoints'
    'AICK orb max 100 keypoints'
]

%%
figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');

data = varyKeypointsDataOrb;
names = varyKeypointsNameOrb;
toshow = 1:size(data,1);
for id=1:size(toshow,2)
    i = toshow(id);
    datavec = reshape(data(i,:,:),size(data,2),size(data,3));
    plot(thresholds(1:max_trid),datavec(step,(1:max_trid)),plotcol(id,:),'LineWidth',2.0);
    
end
legend(names(toshow,:),'Location','SouthEast')

figure()
axis([0 1 0 1.01])
hold on
title(['Translation error <' num2str(thresholds(threshold_id))])
xlabel('time between frames [s]');
ylabel('success ratio');
for id=1:size(toshow,2)
    i = toshow(id);
    datavec = reshape(data(i,:,:),size(data,2),size(data,3));
    d = datavec(:,threshold_id);
    plot((1:size(d,1))/30,d,plotcol(id,:),'LineWidth',2.0);
end
legend(names(toshow,:),'Location','NorthEast')