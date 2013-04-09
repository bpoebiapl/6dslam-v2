disp('generating EMCR plots')
run good_orb_data
run good_orb_data2
run good_surf_data
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

aick_orb        = originalAICKorb_0_AICK_mat_pos;
aick_orb_time   = originalAICKorb_0_AICK_avg_time;

aick_surf       = originalAICKsurf_0_AICK_mat_pos;
aick_orb_time   = originalAICKsurf_0_AICK_avg_time;

ndt             = GICPandNDTroom1_0_NDTMatcher_mat_pos;
ndt_time        = GICPandNDTroom1_0_NDTMatcher_avg_time;

gicp            = GICPandNDTroom1_1_BasicGIcpMatcher_mat_pos;
gicp_time       = GICPandNDTroom1_1_BasicGIcpMatcher_avg_time;

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

%%
varyIterationsDataOrb = [];
varyIterationsDataOrb(1,:,:)=originalAICKorbIterations_0_AICK_mat_pos; 
varyIterationsDataOrb(2,:,:)=originalAICKorbIterations_1_AICK_mat_pos; 
varyIterationsDataOrb(3,:,:)=originalAICKorbIterations_2_AICK_mat_pos; 
varyIterationsDataOrb(4,:,:)=originalAICKorbIterations_3_AICK_mat_pos; 
varyIterationsDataOrb(5,:,:)=originalAICKorbIterations_4_AICK_mat_pos;
varyIterationsDataOrb(6,:,:)=originalAICKorbIterations_5_AICK_mat_pos;


varyIterationsNameOrb = [
    'AICK orb 25 iterations'
    'AICK orb 15 iterations'
    'AICK orb 10 iterations'
    'AICK orb 7 iterations '
    'AICK orb 4 iteration  '
    'AICK orb 1 iteration  '
]

%%
figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');

data = varyIterationsDataOrb;
names = varyIterationsNameOrb;
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

%%
varyKeypointsDataSurf = [];
varyKeypointsDataSurf(1,:,:)=originalAICKsurfKeyPoints_0_AICK_mat_pos; 
varyKeypointsDataSurf(2,:,:)=originalAICKsurfKeyPoints_1_AICK_mat_pos; 
varyKeypointsDataSurf(3,:,:)=originalAICKsurfKeyPoints_2_AICK_mat_pos; 
varyKeypointsDataSurf(4,:,:)=originalAICKsurfKeyPoints_3_AICK_mat_pos; 
varyKeypointsDataSurf(5,:,:)=originalAICKsurfKeyPoints_4_AICK_mat_pos; 
varyKeypointsDataSurf(6,:,:)=originalAICKsurfKeyPoints_5_AICK_mat_pos;


varyKeypointsNameSurf = [
    'AICK surf max 400 keypoints'
    'AICK surf max 300 keypoints'
    'AICK surf max 250 keypoints'
    'AICK surf max 200 keypoints'
    'AICK surf max 150 keypoints'
    'AICK surf max 100 keypoints'
]

%%
figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');

data = varyKeypointsDataSurf;
names = varyKeypointsNameSurf;
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

%%
varyIterationsDataSurf = [];
varyIterationsDataSurf(1,:,:)=originalAICKsurfIterations_0_AICK_mat_pos; 
varyIterationsDataSurf(2,:,:)=originalAICKsurfIterations_1_AICK_mat_pos; 
varyIterationsDataSurf(3,:,:)=originalAICKsurfIterations_2_AICK_mat_pos; 
varyIterationsDataSurf(4,:,:)=originalAICKsurfIterations_3_AICK_mat_pos; 
varyIterationsDataSurf(5,:,:)=originalAICKsurfIterations_4_AICK_mat_pos; 


varyIterationsNameSurf = [
    'AICK surf 25 iterations'
    'AICK surf 7 iterations '
    'AICK surf 5 iterations '
    'AICK surf 3 iterations '
    'AICK surf 1 iteration  '
]

%%
figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');

data = varyIterationsDataSurf;
names = varyIterationsNameSurf;
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

%%
varyWordThresholdDataSurf = [];
varyWordThresholdDataSurf(1,:,:) =bowAICKsurf_wordthreshold_bl_200_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(2,:,:) =bowAICKsurf_wordthreshold_bl_205_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(3,:,:) =bowAICKsurf_wordthreshold_bl_210_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(4,:,:) =bowAICKsurf_wordthreshold_bl_215_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(5,:,:) =bowAICKsurf_wordthreshold_bl_220_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(6,:,:) =bowAICKsurf_wordthreshold_bl_225_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(7,:,:) =bowAICKsurf_wordthreshold_bl_230_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(8,:,:) =bowAICKsurf_wordthreshold_bl_235_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(9,:,:) =bowAICKsurf_wordthreshold_bl_240_0_BowAICKv2_mat_pos; 
varyWordThresholdDataSurf(10,:,:) =bowAICKsurf_wordthreshold_bl_245_0_BowAICKv2_mat_pos; 
varyWordThresholdDataSurf(11,:,:) =bowAICKsurf_wordthreshold_bl_250_0_BowAICKv2_mat_pos; 
varyWordThresholdDataSurf(12,:,:) =bowAICKsurf_wordthreshold_bl_255_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(13,:,:) =bowAICKsurf_wordthreshold_bl_260_0_BowAICKv2_mat_pos; 
varyWordThresholdDataSurf(14,:,:) =bowAICKsurf_wordthreshold_bl_265_0_BowAICKv2_mat_pos; 
varyWordThresholdDataSurf(15,:,:) =bowAICKsurf_wordthreshold_bl_270_0_BowAICKv2_mat_pos; 
varyWordThresholdDataSurf(16,:,:) =bowAICKsurf_wordthreshold_bl_275_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(17,:,:)=bowAICKsurf_wordthreshold_bl_280_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(18,:,:) =bowAICKsurf_wordthreshold_bl_285_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(19,:,:) =bowAICKsurf_wordthreshold_bl_290_0_BowAICKv2_mat_pos;
varyWordThresholdDataSurf(20,:,:) =bowAICKsurf_wordthreshold_bl_295_0_BowAICKv2_mat_pos;
%varyWordThresholdDataSurf(21,:,:)=bowAICKsurf_wordthreshold_bl_300_0_BowAICKv2_mat_pos; 


varyWordThresholdNameSurf = [
    'BowAICK surf word threshold 0.200'
    'BowAICK surf word threshold 0.205'
    'BowAICK surf word threshold 0.210'
    'BowAICK surf word threshold 0.215'
    'BowAICK surf word threshold 0.220'
    'BowAICK surf word threshold 0.225'
    'BowAICK surf word threshold 0.230'
    'BowAICK surf word threshold 0.235'
    'BowAICK surf word threshold 0.240'
    'BowAICK surf word threshold 0.245'
    'BowAICK surf word threshold 0.250'
    'BowAICK surf word threshold 0.255'
    'BowAICK surf word threshold 0.260'
    'BowAICK surf word threshold 0.265'
    'BowAICK surf word threshold 0.270'
    'BowAICK surf word threshold 0.275'
    'BowAICK surf word threshold 0.280'
    'BowAICK surf word threshold 0.285'
    'BowAICK surf word threshold 0.290'
    'BowAICK surf word threshold 0.295'
%    'BowAICK surf word threshold 0.30'
]
%%
figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');

data = varyWordThresholdDataSurf;
names = varyWordThresholdNameSurf;
toshow = 1:4:size(data,1);
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

%%
varyWordThresholdDataOrb = [];
                                 %bowAICKorb_wordthreshold_bl_300_0_BowAICKv2_mat_pos
varyWordThresholdDataOrb(1,:,:)  =bowAICKorb_wordthreshold_bl_300_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(2,:,:)  =bowAICKorb_wordthreshold_bl_310_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(3,:,:)  =bowAICKorb_wordthreshold_bl_320_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(4,:,:)  =bowAICKorb_wordthreshold_bl_330_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(5,:,:)  =bowAICKorb_wordthreshold_bl_340_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(6,:,:)  =bowAICKorb_wordthreshold_bl_350_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(7,:,:)  =bowAICKorb_wordthreshold_bl_360_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(8,:,:)  =bowAICKorb_wordthreshold_bl_370_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(9,:,:)  =bowAICKorb_wordthreshold_bl_380_0_BowAICKv2_mat_pos;
varyWordThresholdDataOrb(10,:,:) =bowAICKorb_wordthreshold_bl_390_0_BowAICKv2_mat_pos;




varyWordThresholdNameOrb = [
    'BowAICK orb word threshold 300'
    'BowAICK orb word threshold 310'
    'BowAICK orb word threshold 320'
    'BowAICK orb word threshold 330'
    'BowAICK orb word threshold 340'
    'BowAICK orb word threshold 350'
    'BowAICK orb word threshold 360'
    'BowAICK orb word threshold 370'
    'BowAICK orb word threshold 380'
    'BowAICK orb word threshold 390'
%    'BowAICK orb word threshold 300'
]
%%
figure()
axis([0 max(thresholds(1:max_trid)) 0 1.01])
hold on
title('Translation performance')
xlabel('threshold [m]');
ylabel('success ratio');

data = varyWordThresholdDataOrb;
names = varyWordThresholdNameOrb;
toshow = 1:4:size(data,1);
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