# Ball on Plate Balancing
![Output sample](https://github.com/avrech/ball_on_plate_balancing/blob/master/images/ezgif.com-video-to-gif.gif)

In this project we implemented a digital PID controller for ball on plate balancing.  
The work done by [Daniela Ben-David](https://www.linkedin.com/in/daniela-ben-david-9852b6164/) and [Avrech Ben-David](https://www.linkedin.com/in/avrech-ben-david-948022117/) in the CRML Lab, Technion, under the supervision of Dr. Ilan Rusnak.  
A link to the project page on facebook:  
https://www.facebook.com/kohaik/videos/10212951342449166/?t=0

## Hardware
The system consists of:  
1. Arduino Nano board equipped with ATmega328 microprocessor  
2. Stewart Platform  
3. 6 TowerPro MG996R Servo motors feeded by external 5V power supply
4. Touchpad  


## The Control Loop
![Alt text](images/close-loop-control.png)    
The ball position is measured by the touchpad, and translated to (x,y) coordinates.
Then, the PID controller computes the desired plate angles (roll, pitch) which will bring the ball to the origin.
These angles are translated to the desired motor angles, (ideally in a closed form) using the inverse kinematics 
of the stewart platform. 
The Arduino controller set the new motor angles, and the ball moves to its next position. 
The process repeats 50 times per second, until the ball arrives at the target. 

## Tricks to make things working  
### Dealing with non-ideal sensors
The main difficult in developing "real" systems is that things do not work ideally.   
In our case, the "ball on plate" problem turned out to be "extreme noisy sensors" problem.   
The PID controller by itself is robust enough to balance the ball, had its input be a clear signal.   
Unfortunately, it is not the case.  

In order to reduce the noise effect we take the following steps:  
1. Reduce the noise in its pysical source as much as possible.   
2. Filter the majority of outliers by thresholding.  
3. Smoothen the "almost clean" signal using standard filters (e.g IIR).    

First we double the voltage settling time of the touchpad.  
We look at the resulting signal, and see that there is still a considerable mass of outliers:  
![Alt text](images/noisy-measurment.png?raw=true "A Noisy Position Signal vs. Time")  

Based on these samples, the ball's velocity (first derivative of the position) looks like this:  
![Alt text](images/x-derivative.png?raw=true,center=true "A Noisy Position Signal vs. Time")  
This noisy signal confuses the PID controller, which becomes completely crazy.  
No LPF can deal with such a garbage, but simple thresholding can.  

The trick here is a simple re-sampling. We compute the distance between consequent samples, and if it is too large, we sample again. This almost eliminates the noise:  
![Alt text](images/after-resampling.png?raw=true,center=true "A Noisy Position Signal vs. Time")  

The rest of the work can be done by a standard butterworth filter.    
![Alt text](images/after-butterworth.png?raw=true,center=true "A Noisy Position Signal vs. Time")  

### Dealing with numerical instabilities
The inverse kinematics of the stewart platform is subject to numerical issues (e.g dividing by small numbers). 
Following [this](https://github.com/ThomasKNR/RotaryStewartPlatform/blob/master/src_arduino_code/platform.ino) we compute the motor angles using binary search:  
We compute the Stewart platform leg lengths like they where telescopic legs.  
Then, for each motor, we binary-search for the optimal angle, which minimizes the error between the desired leg length, and the actual distance between the motor's joint and the plate joint, had we set the particular angle.  

## Reference
### Stewart Platform & Servo Motors Mathematics  
https://web.archive.org/web/20130506134518/http://www.wokinghamu3a.org.uk/Maths%20of%20the%20Stewart%20Platform%20v5.pdf

### Ball on Plate Kinematics  
The ball on plate system can be decomposed into two orthogonal ball on beam systems.  
The kinematic equations as well as the transfer function can be found at  
http://ctms.engin.umich.edu/CTMS/index.php?example=BallBeam&section=SystemModeling  
