
% EMG1f=highpass(RawEMG,10,512);
actual_time=length(RawEMG);

% subplot(2,1,1);
figure(1);
plot(Time(1:actual_time),RawEMG);
title('Filtered Data');
xlabel('time(s)','FontSize',18,'FontWeight','bold');
ylabel('amplitude(mV)','FontSize',18,'FontWeight','bold');
ylim([0 1]);
xlim([0 Time(actual_time)]);

% figure(2);
% rmsplot=rms2(RawEMG,50,0,0);
% new_l=length(rmsplot);
% rms_time=linspace(Time(1),Time(actual_time),new_l);
% plot(rms_time,rmsplot);
% title('RMS plot');
% xlabel('time(s)','FontSize',18,'FontWeight','bold');
% ylabel('amplitude(V)','FontSize',18,'FontWeight','bold');
% ylim([0 1]);
% xlim([0 rms_time(end)]);

% subplot(2,1,2);
% plot(Time(1:actual_time),EMG1f);
% title('Filtered data');
% xlabel('time(s)');
% ylabel('amplitude(V)');
% ylim([-0.3 0.3]);
% xlim([0 Time(actual_time)]);