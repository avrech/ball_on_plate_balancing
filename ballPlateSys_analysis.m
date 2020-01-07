% Analize ball on plate system
clear all; close all; clc;
load('ballPlateSys_analysis_data.mat');
%% 1. plot error signal
figure(1)
title('error record 50Hz');
subplot(1,2,1);
plot(rec_e_50hz_p06(1:2:500),'.r');
ylabel('x error [cm]');
subplot(1,2,2);
plot(rec_e_50hz_p06(2:2:500),'.b');
ylabel('y error [cm]');

%% 2. plot ball x coordinate:
figure(2)
plot(rec_x_50hz_p06_d02,'.');
title(sprintf('ball x coordinate @ 50Hz \n K_p = 0.8 , K_d = 0.2'));
ylabel('x [cm]');
xlabel('sampling time');
x_len=length(rec_x_50hz_p06_d02);
%% 3. 1st order iir with tau = 0.2
x_iir = filter([1],[1 -0.2],rec_x_50hz_p06_d02);
figure(3)
plot(rec_x_50hz_p06_d02,'.')
hold on
plot(x_iir,'.')
title('1^{st} order IIR, \tau = 0.2');
legend('x','x_{iir}')


%% 4. butterworth
fc = 5; % [Hz]
fs = 50;% [Hz]
[b3,a3] = butter(3,fc/(fs/2));
[b10,a10] = butter(10,fc/(fs/2));
figure(40)
freqz(b3,a3)
hold on
freqz(b10,a10) 
title('Butterworth IIR Filters');
legend('Butter3', 'Butter10');
hold off
x_butter3 = filter(b3,a3,rec_x_50hz_p06_d02);
x_butter10 = filter(b10,a10,rec_x_50hz_p06_d02);

figure(41)
subplot(3,1,1);
plot(rec_x_50hz_p06_d02,'.')
title('The noisy signal');
subplot(3,1,2);
plot(x_butter3,'.')
title('Filtering with 3^{rd} order butterworth');
subplot(3,1,3);
plot(x_butter10,'.')
title('Filtering with 10^{th} order butterworth');

figure(42)
subplot(1,2,1);
plot(rec_x_50hz_p06_d02,'.')
hold on
plot(x_butter10)
hold off
title('Filtering with 10^{th} order butterworth');
legend('Noisy Signal','Filtered')
xlabel('Time')
ylabel('Position [cm]')
%% 5. remez
f = [0 0.1 0.2 1];
a = [1.0 1.0 0.0 0];
remez5 = firpm(5,f,a);
remez10 = firpm(10,f,a);
figure(50)
[h,w] = freqz(remez5,1,512);
plot(f,a,w/pi,abs(h))
hold on
[h,w] = freqz(remez10,1,512);
plot(f,a,w/pi,abs(h))
title('Remez FIR Filters')
legend('Ideal','remez5','remez10')
xlabel 'Radian Frequency (\omega/\pi)', ylabel 'Magnitude'

x_remez5 = conv(remez5,rec_x_50hz_p06_d02);
x_remez10 = conv(remez10,rec_x_50hz_p06_d02);


figure(42)
subplot(1,2,2);
plot(rec_x_50hz_p06_d02,'.b')
hold on
plot(x_remez10,'g')
legend('x','remez 10');
title('Filtering with 10^{th} order Remez');
legend('Noisy Signal','Filtered')
xlabel('Time')
ylabel('Position [cm]')
%% 6. custom selective filter
x_derivative = rec_x_50hz_p06_d02(2:end)- rec_x_50hz_p06_d02(1:end-1);
outliers_idx = find(abs(x_derivative)>1);
figure(60)
subplot(1,2,1)
plot(x_derivative,'.')
hold on; plot(ones(x_len)); plot(-1*ones(x_len)); hold off
title('x derivative')
xlabel('time');
ylabel('[cm/T_s]');
subplot(1,2,2)
hist(x_derivative,30)
title('dx histogram');
xlabel('dx/dt')
ylabel('amount')
%% 7. recover x:
x_est = rec_x_50hz_p06_d02(1);
dx = 0;
max_consequitive_outliers = 5;
prev_dx = 0;
dx_th = 1.1;
for idx = 2:x_len
    dx = rec_x_50hz_p06_d02(idx)-x_est(end);
    if abs(dx) < dx_th % this is inlier
        x_est = [x_est rec_x_50hz_p06_d02(idx)];
        ddx = dx - prev_dx;
    else
        % x_before_filter(idx) is outlier.
        % estimate x
        dx_est = x_est(end)-x_est(length(x_est)-1) + ddx;
        est = x_est(end) + dx_est;
        x_est =  [x_est est];
%         x_est =  [x_est x_est(end)];
    end 
end
figure(70)
plot(rec_x_50hz_p06_d02(2:end),'.b');
hold on
plot(x_est(1:end),'r');
hold off
legend('x','filtered x');
title(sprintf('Decision Based Non-Linear Filter\nDerivative Threshold = 1.1[cm/T_s]'));
xlabel('Time');
ylabel('[cm]')
%% 8. max consequtive outliers:
x_est = rec_x_50hz_p06_d02(1);
dx = 0;
max_consequitive_outliers = 100;
prev_dx = 0;
dx_th = 1.1;
for idx = 2:x_len
    dx = rec_x_50hz_p06_d02(idx)-x_est(end);
    if abs(dx) > dx_th && outlier_cnt > 0% this is outlier
        x_est =  [x_est x_est(end)];
        outlier_cnt = outlier_cnt - 1;
    else 
        x_est = [x_est rec_x_50hz_p06_d02(idx)];
        outlier_cnt = max_consequitive_outliers; % init cnt.        
    end 
end
figure(80)
plot(rec_x_50hz_p06_d02(2:end),'.b');
hold on
plot(x_est(1:end),'r');
hold off
legend('x','filtered x');
title(sprintf('Decision Based Non-Linear Filter\ndx/dt Threshold = 1.1[cm/T_s], Max Outliers 100'));
xlabel('Time');
ylabel('[cm]')
%% compare three filters:
figure(71)
subplot(4,1,1);
plot(rec_x_50hz_p06_d02,'.b')
title('The noisy signal');
subplot(4,1,2);
plot(x_butter10,'.r')
title('Filtering with 10^{th} order butterworth');
subplot(4,1,3);
plot(x_remez10,'.g')
title('Filtering with 10^{th} order remez');
subplot(4,1,4);
plot(x_est(1:end),'.');
title('Derivative threshold decision');

%% 8. plot ball position
figure(8)
subplot(1,2,1);
plot(rec_ball_position_50hz_p06_d02(1:2:100),'.r');
title(sprintf('ball position @ 50Hz \n K_p = 0.8 , K_d = 0.2'));
ylabel('x [cm]');
subplot(1,2,2);
plot(rec_ball_position_50hz_p06_d02(2:2:100),'.b');
title('ball position @ 50Hz\nK_p = 0.8 , K_d = 0.2');
ylabel('y [cm]');

%% 9. psd:
rng default
Fs = 50;
t = 0:1/Fs:1-1/Fs;
% Obtain the periodogram using fft. 
% The signal is real-valued and has even length. 
% Because the signal is real-valued, you only need power 
% estimates for the positive or negative frequencies. 
% In order to conserve the total power, multiply all 
% frequencies that occur in both sets 
% -- the positive and negative frequencies -- by a factor of 2. 
% Zero frequency (DC) and the Nyquist frequency do not occur twice. Plot the result.
x = rec_e_50hz_p06;
figure(90)
periodogram(x,rectwin(length(x)),length(x))

x = rec_e_50hz_p06_d02;
figure(91)
periodogram(x,rectwin(length(x)),length(x))

x = rec_ang_50hz_p06;
figure(92)
periodogram(x,rectwin(length(x)),length(x))

x = rec_ang_50hz_p06_d02;
figure(93)
periodogram(x,rectwin(length(x)),length(x))

