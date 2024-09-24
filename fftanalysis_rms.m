X=RawEMG;

L=length(Time);
Fs=L/Time(end);
T=1/Fs;
t=(0:L-1)*T;


Y=fft(X);

P2 = abs(Y/L);
P1 = P2(1:(L)/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:((L)/2))/L;
figure
plot(f,P1) 
xlabel('frequency(Hz)','FontSize',18,'FontWeight','bold');
ylabel('power(dB)','FontSize',18,'FontWeight','bold');

% X2=Filtered_data;
% 
% 
% 
% Y2=fft(X2);
% 
% P2 = abs(Y2/L);
% P1 = P2(1:(L)/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:((L)/2))/L;
% figure
% plot(f,P1) 
% xlabel('frequency(Hz)','FontSize',18,'FontWeight','bold');
% ylabel('power(dB)','FontSize',18,'FontWeight','bold');



% EMG1f=highpass(RawEMG,10,512);
% X=EMG1f;
% 
% L=length(Time);
% Fs=L/Time(end);
% T=1/Fs;
% t=(0:L-1)*T;
% 
% 
% Y=fft(X);
% 
% P2 = abs(Y/L);
% P1 = P2(1:(L)/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:((L)/2))/L;
% figure
% plot(f,P1) 
% 
%  title("Single-Sided Amplitude Spectrum of X(t)")
%  xlabel("f (Hz)")
% ylabel("|P1(f)|")
% 
% d = designfilt('bandstopiir','FilterOrder',2, ...
%                'HalfPowerFrequency1',59,'HalfPowerFrequency2',61, ...
%                'DesignMethod','butter','SampleRate',Fs);
% X1 = filtfilt(d,X);
% 
% d2 = designfilt('bandstopiir','FilterOrder',2, ...
%                'HalfPowerFrequency1',119,'HalfPowerFrequency2',121, ...
%                'DesignMethod','butter','SampleRate',Fs);
% d3 = designfilt('bandstopiir','FilterOrder',2, ...
%                'HalfPowerFrequency1',179,'HalfPowerFrequency2',181, ...
%                'DesignMethod','butter','SampleRate',Fs);
% d4 = designfilt('bandstopiir','FilterOrder',2, ...
%                'HalfPowerFrequency1',239,'HalfPowerFrequency2',241, ...
%                'DesignMethod','butter','SampleRate',Fs);
% d5 = designfilt('bandpassfir','FilterOrder',2, ...
%                'CutoffFrequency1',10,'CutoffFrequency2',200, ...
%                'SampleRate',Fs);
% X2=filtfilt(d2,X1);
% X3=filtfilt(d3,X2);
% X4=filtfilt(d4,X3);
% X5=filtfilt(d5,X4);
% figure, plot(RawTime,X);
% figure, plot(RawTime,X5);
% noise=X-X5;
% ratio=snr(X,noise);
% disp(ratio);

% window_length=round(0.05*Fs);
% signal_rms=rms2(X5,window_length,0,0);
% time_fake=L/(length(signal_rms)*Fs);
% time2=1:length(signal_rms);
% time2=time2*time_fake;
% figure, plot(time2,signal_rms);

% Y2=fft(X5);
% P3 = abs(Y2/L);
% P4 = P3(1:(L)/2+1);
% P4(2:end-1) = 2*P4(2:end-1);
% f = Fs*(0:((L)/2))/L;
% plot(f,P4) 