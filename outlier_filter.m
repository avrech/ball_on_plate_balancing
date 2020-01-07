clear ; close all; clc
%% load x coordinate 
load('x.mat');
%% 1. plot ball x coordinate:
figure(1)
plot(x,'.');
title(sprintf('Ball x coordinate\nDouble voltage settling time\nUsing non-linear filter'));
ylabel('x [cm]');
xlabel('Time');
x_len=length(x);

%% 2. filter x with butterworth
fc1 = 2; % [Hz]
fc2 = 5;% [Hz]
fs = 50;% [Hz]
[b1,a1] = butter(1,2/(fs/2));
[b2,a2] = butter(1,5/(fs/2));
figure(21)
freqz(b1,a1)
title('1^{st} Order Butterworth f_c = 2[Hz]');
figure(22)
freqz(b2,a2) 
title('1^{st} Order Butterworth f_c = 5[Hz]');

x_butter1 = filter(b1,a1,x);
x_butter2 = filter(b2,a2,x);

figure(23)
plot(x,'.b')
hold on
plot(x_butter1,'.g')
plot(x_butter2,'.r')
legend('x','butter1 @ 2Hz','butter1 @ 5Hz');
hold off
title('Butterworth IIR Filters')
xlabel('time');
ylabel('[cm]');

figure(24)
periodogram(x_butter2,rectwin(length(x_butter2)),length(x_butter2))
figure(25)
periodogram(x,rectwin(length(x)),length(x))

%% 3. filter x with remez
f = [0 0.1 0.2 1];
a = [1.0 1.0 0.0 0];
remez5 = firpm(5,f,a);
remez10 = firpm(10,f,a);
figure(31)
[h,w] = freqz(remez5,1,512);
plot(f,a,w/pi,abs(h))
hold on
[h,w] = freqz(remez10,1,512);
plot(f,a,w/pi,abs(h))
legend('Ideal','remez5','remez10')
xlabel 'Radian Frequency (\omega/\pi)', ylabel 'Magnitude'

x_remez5 = conv(remez5,x);
x_remez10 = conv(remez10,x);


figure(32)
plot(x,'.b')
hold on
plot(x_remez5,'.g')
plot(x_remez10,'.r')
legend('x','remez 5','remez 10'); 
title('Remez FIR')
xlabel('time');
ylabel('[cm]');
%% 4. final balancing results:
load('final_xy.mat');
final_x = final_xy(531:2:end);
final_y = final_xy(532:2:end);
time = (1:length(final_x))*0.02; % time line in seconds.
figure(4)
plot(time,final_x,'b')
hold on
plot(time,final_y,'g')
hold off
legend('x','y');

title(sprintf('Ball on Plate Balancing Final Results\nRef Position = (0,-3)[cm]\nF_s = 50[Hz]\nK_P = -0.6[rad/cm]\nK_I = -0.01[rad/(cm*sec)]\nK_D = -0.2[rad*sec/cm]\ndSample th = 40[bins]\nMax Sampling Attempts = 10\n'));
xlabel('time [sec]');
ylabel('[cm]');
