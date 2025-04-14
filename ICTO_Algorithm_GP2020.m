%% INPUTS
% fo: filter oder
% ssf: recording frequency
% Rawdata: Gyroscope data (time series)
%
%% OUTPUTS
% ICs: Initial contact indices
% TOs: Toe off indices
%
%% Event detection algorithm used for:
% Fadillioglu, C.*, Stetter, B. J.*, Ringhof, S., Krafft, F. C., Sell, S., & Stein, T. (2020). 
% Automated gait event detection for a variety of locomotion tasks using a novel gyroscope-based algorithm. 
% Gait & Posture, 81(June), 102–108. https://doi.org/10.1016/j.gaitpost.2020.06.019
%%

function [ICs, TOs]=EventDetection(fo, ssf, RawData)

cof=15;
[output1,output2]=butter(fo,cof/ssf);
GYROUS_15=filtfilt(output1,output2,RawData);

%% Complementary Signal for TO detection

cof=10;
[output1,output2]=butter(fo,cof/ssf);

ABS1=RawData-GYROUS_15;
CompSig=filtfilt(output1,output2,ABS1);

%% Swing Phase prediction

[~,LOC]=findpeaks(GYROUS_15,'MinPeakHeight',3000,'MinPeakDistance',500);

%% IC detection

indIC=1;
data_d=detrend(GYROUS_15)

for kk=1:numel(LOC)-1

    IC_rel(indIC,1)=find(data_d(LOC(kk):LOC(kk)+1500)<0,1,'first');
    ICs(indIC,1)=IC_rel(indIC,1)+LOC(kk)-1;
    indIC=indIC+1;
end

%% TO detection

indTO=1;

for kk=2:numel(LOC)

    if  LOC(kk)-LOC(kk-1)<3000 %to check if the peak is just in the beginning

        Y=LOC(kk)-LOC(kk-1);

        %finds a minimum for the second search in the following step
        [~,loc_min1]=min(GYROUS_15(LOC(kk)-round(Y/2):LOC(kk)-round(Y/10)));
        IND_temp=loc_min1+LOC(kk)-round(Y/2)-1;

        if Y<1500 %to distunguish between fast and slow events
            [~,loc_min2]=max(CompSig(LOC(kk)-round(Y/2):IND_temp));

        else
            [~,loc_min2]=min(CompSig(LOC(kk)-round(Y/2):IND_temp+round(Y/10)));

        end

        TOs(indTO,1)=loc_min2+LOC(kk)-round(Y/2)-1;
       
        indTO=indTO+1;

    end
end

end