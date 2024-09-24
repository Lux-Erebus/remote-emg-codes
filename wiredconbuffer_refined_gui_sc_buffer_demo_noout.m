
%%remove buffer variables if it works without them, useless computation
close all
clear variables


% clear act
arduinoObj = serialport("COM5",9600); %%read objectno
prompt='sampling frequency in numerals?';
frequency=input(prompt);
timespace=round(1/frequency,3,"significant");
%gripper=serialport('COM21',9600);
% gripper=arduino("COM21",'uno');
% act=serialport("COM10", 9600);  
% writeDigitalPin(gripper,"D3",1);
% writeDigitalPin(gripper,"D4",0);
%%intialization variables%%n
buffer=0.2; %%seconds between each plot function
filter_buffer=0.1; %%seconds used for filter buffering
plot_size=3500; %%number of samples in a plot
%d=input('duration required? ','s');
%dur=str2double(d);hh
% dur=30; %%duration of record

configureTerminator(arduinoObj,"CR/LF"); %%terminator read
flush(arduinoObj); %%clear object each run to reset port
figure;
fig = uifigure("Position",[1000, 500, 200, 50]);
arduinoObj.UserData = struct("Data",[],"RAWData",[],"Data_rms",[],"Baseline",[],"Baseline_avg",0,"Baseline_sd",0,"Count",0,"Time",[],"timelim",buffer,"timelim_filt",filter_buffer,"timer",0,"timegap",timespace,"DataBuffer",[],"ValBuffer",[],"prevtime",0,"plotsize",plot_size,"threshold",0,"firstime",0,"end",0,"fig",fig,"fs",frequency);

btn = uibutton(fig,'push', 'Text','Stop','Position',[100, 25, 100, 25],...
               'ButtonPushedFcn', @(btn,event) ender(arduinoObj));
tic;
configureCallback(arduinoObj,"terminator",@plotData); %%function to call each time data is detected


function plotData(src, ~)

% Read the ASCII data from the serialport object.
data = readline(src); 
val=str2double(data);
% Convert the string data to numeric type and save it in the UserData
% property of the serialport object.
if(isnan(val)) %%reset for next read if value is trash
    return
end  
val2=round((((val/15925248)-0.5)*2*2.4/3.5),4,"significant");
%%val2=round(val*3.3/4096,4,'significant');

src.UserData.DataBuffer(end+1) = val2; %% complete data array into which value is stored
%src.UserData.DataBuffer(end+1) = val; %% data buffer array into which value is stored, buffer_dur*fs in size
temptime=src.UserData.Count*src.UserData.timegap; %%time of operation
src.UserData.Time(end+1)=temptime; %% time array for storing value

temptime_buffer=(length(src.UserData.DataBuffer))*src.UserData.timegap;%%buffer timeline

%src.UserData.ValBuffer(end+1)=temptime; %% time buffer array for mapping
%deltime=temptime-src.UserData.prevtime; %%difference in time since previous operation
%src.UserData.prevtime=temptime; %% save new time
src.UserData.Count = src.UserData.Count + 1; %% helps figure the starting time for the data to be plotted
src.UserData.timer=src.UserData.timer+src.UserData.timegap; %% timer to figure out buffer

if(temptime_buffer>src.UserData.timelim_filt)
    emg_filtered=highpass(src.UserData.DataBuffer,10,src.UserData.fs); %filter the buffer
    src.UserData.RAWData=cat(2,src.UserData.RAWData, src.UserData.DataBuffer); %append raw data
    %disp("buffer length:");
    %disp(length(src.UserData.DataBuffer));
    src.UserData.Data=cat(2,src.UserData.Data,emg_filtered); %append filtered data
    src.UserData.Data_rms=cat(2,src.UserData.Data_rms,rms(emg_filtered)); %find rms and add the value
    src.UserData.DataBuffer=[]; 
    %disp("time length:");
    %disp(length(src.UserData.Time));
    %disp("data length:");
    %disp(length(src.UserData.RAWData));
    %disp(length(src.UserData.Data));

    if src.UserData.firstime==0 && temptime>7 %threshold calculation
        src.UserData.Baseline=src.UserData.Data_rms;
        src.UserData.firstime=1;
        src.UserData.Baseline_avg=mean(src.UserData.Baseline);
        src.UserData.Baseline_sd=std(src.UserData.Baseline);
        disp("threshold calculated:");
        disp(src.UserData.Baseline_avg);
        disp(src.UserData.Baseline_sd);
    end

    if src.UserData.firstime==1 && (src.UserData.Data_rms(end)>(src.UserData.Baseline_avg+6*src.UserData.Baseline_sd))
        disp("1");
        %write(src.UserData.gripper,1,'int32');
%         writeDigitalPin(src.UserData.gripper,"D2",1);
        
    elseif src.UserData.firstime==1
        disp("0");
        %write(src.UserData.gripper,0,'int32');
%         writeDigitalPin(src.UserData.gripper,"D2",0);
    
    end

    if src.UserData.timer >= src.UserData.timelim  %%only entered after buffer duration
        src.UserData.timer=0; %%reset buffer clock timer
        if src.UserData.Count < src.UserData.plotsize %%plot after number of samples attained
            plot(src.UserData.Time,src.UserData.Data);
            title('EMG Data');
            xlabel('Time(s)');
            ylabel('Amplitude(V)');
            ylim([-0.3 0.3])
            xlim([0 7])    
            drawnow;
        end
        if src.UserData.Count > src.UserData.plotsize %%plot after number of samples attained
            plot(src.UserData.Time(end-int32(src.UserData.plotsize+1):end),src.UserData.Data(end-int32(src.UserData.plotsize+1):end));
            title('EMG Data');
            xlabel('Time(s)');
            ylabel('Amplitude(V)');
            ylim([-0.3 0.3])
            xlim([src.UserData.Time(end-int32(src.UserData.plotsize+1)) src.UserData.Time(end)])    
            drawnow;
        end
    end
end

    if src.UserData.end==1
        toc
        configureCallback(src, "off");
        saver(src)
    
    end
end

function saver(src)
%     b=struct("Data",[],"Time",[]);
    AllData=src.UserData;
        RawEMG=AllData.RAWData;
        Time=AllData.Time;
        Baseline=AllData.Baseline;
        RMS= AllData.Data_rms;
        Filtered_data=AllData.Data;

    prompt='file name?';
    fname=input(prompt,"s");
%     fname=join(["Data\",x],"");
    save(['D:\Documents\MATLAB\BCI_data\' fname],'RawEMG', "Time","Filtered_data","Baseline","RMS");
    delete(src.UserData.fig);
end

function ender(src)
    disp("ended")
    src.UserData.end=1;
    
end

