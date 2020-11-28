
# 1 OLDX-MocoMoco Quadruped Robot Development Platform Project
<div align=center><img width="600" height="130" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/logo.JPG"/></div>
<div align=center><img width="400" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/fc2.jpg"/></div>

  ____Moco8 Quadruped is a fully opensourced project.With the continuous introduction of foot robots by Boston Dynamics in recent years,
  it has attracted the attention of many fans at home and abroad. By combining it with the developed flight control project, the MocoMoco quadruped robot development platform was launched, including the robot structure, gait algorithm and SLAM. The project follows the GPL agreement and can modify and re-develop the relevant source code in the DEMO.<br><br>

**Relevant information** 
Robot test video update address：https://www.bilibili.com/video/av46405055 <br>
To facilitate the program to download the current versions of the firmware separately from here：https://github.com/golaced/MocoMoco_Software<br>
Robot calibration video tutorial：https://www.bilibili.com/video/av51603154<br>
<br>
**-If this project is helpful to you, please Star our project-**<br>
**-If you are willing to share the optimization and improvement of this project, please contact golaced@163.com**<br>

<br>
# 2 Introduction to MocoMoco Quadruped Robot Platform

 <div align=center><img width="550" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/r2.jpg"/></div>

____The MocoMoco quadruped robot is an 8-DOF foot robot which borrows from the Minitature robot introduced by GhostRobtic，The reason why the 12-degree-of-freedom robot is not adopted like Boston Dynamics or other developers is because its maintenance difficulty and cost are greatly improved.
At the same time, for small-footed robots, compared with their own moving speed, there is no absolute narrow area in the external environment, which can meet the reliable movement of most indoor and outdoor scenes.<br>
____In addition, The circuit board hardware adopts an integrated integrated design. The smallest Raspberry Pi A3+ has the characteristics of small size and high performance, which can provide development possibilities for subsequent images and lidar SLAM. The control board uses STM32F4 as the processor, and onboard 3-axis acceleration  The meter, 3-axis gyroscope, 3-axis magnetometer and barometer are designed with internal shock absorption, and the circuit mounting holes can be perfectly installed with the Raspberry Pi while providing a 3D printed shell.The control board uses an external replaceable power supply module for power supply. It has 12 PWM outputs and 4 AD sampling, and has an external expansion of 4 serial ports. It is compatible with the wireless debugging launched by Pioneering Atom. The robot is different in its debugging。

**Controller hardware parameters：**

project|Parameter
-------------|-------------
processor|STM32F405RGT6
Processor performance|32Bit ARM Cortex-M4 168MH
Gyroscope & Accelerometer|LSM6DS33 
Magnetometer|LIS3MDL
Barometer|MS5611
IO port|GPS-1 USART-4 AD-4 Touch IO-4
PWM| 12 Channles 
Powered by|5V-in External power supply for Servo ADSensor 3.3/5V power supply options
Image processor|Raspberry pi A3+  1.4G 4 Core  
Remote control method|2.4G SBUS
Ground station|QGround (Need additional OLDX-REMOTE monitor)

**Robot parameters：**

project|parameter
-------------|-------------
Foot robot type|Parallel robot with 8 degrees of freedom
size|30cm * 20cm *10cm  
Full leg length|8cm
weight|600g
powered by|7.4V  18650 * 2(3000mah) 
Gait support|Tort  Fly-Trot 
Maximum moving speed|0.4m/s
Maximum turning speed|30度/s
Gait cycle|>0.35 s
Servo |Kpower 12g *8(6V-60度/0.035s)
Control mode|Remote control mode


<br>
**Controller PCB interface description：**
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/fc.jpg"/></div>
<br>

interface|IO order(From a picture perspective)|
-------------|-------------|-------------
Reset|Controller reset button|
Wireless download|From left to right：GND SCK SWD 5V|
SWD|From left to right：GND SWD SCK 5V|
GPS|From left to right：5V RX2 TX2 SCL SDA GND|
Foot sensor|From top to bottom： VCC GND AD1/SCL1 KEY_GROUND1/SDA1|
AD Power supply options|top to bottom：3.3V  5V|
SBUS|From left to right：GND 5V Signal|WBUS SBU
    |From left to right: RX4 TX4 GND 5V| 
    |From left to right: TX1 RX1 GND 5V|  
Raspberry pi|From left to right：TX3 RX3 GND 5V|Raspberry Pi A3
PWM-?|From left to right：GND VCC PWM   From top to bottom：Outer Servo  Inner Servo  |
Power supply|From top to bottom：BEEP EN GND GND 5V  VCC   VCC  VBAT|Power module

**Note: The controller body is RGB LED direction. In the above table, only the pin sequence of leg 1 servo interface and sensor interface is given, and the other three legs are mirror-symmetrical structures, that is, taking leg 2 as an example from top to bottom:  From left to right, the gimbal of the inner servo of the outer servo is: PWM VCC GND The sensor of leg 2 is from top to bottom: KEY_GROUND2/SDA2 AD2/SCL2 GND VCC**

# 3 DEMO test
____The project provides a free basic test 3D printer rack that can be downloaded and printed for free. The official will launch a full-carbon version of the rack. Please pay attention to the third-party expansion module launched by Taobao. The controller also serves as the screws required for each version of the rack  Models are as follows：

Metal Parts|Quantity (Official Carbon Rack)|Assembly
-------------|-------------|----------
M2*16|8|Fixed arm
M2*5|8|Install the steering gear
M3*5|8|Mounting leg
M3*10|4|Install the soleM3*32 Copper column|4|Central body support
M3*32 Copper column|4|Central body support
M3*4|8|Central body fixed
M1.4*4|8|Fixed gear
M2.5*3|4|Install battery box
M2.5*5|8|Fixed controller

## 3.1 Controller assembly (Raspberry Pi A3)
(1) Install Raspberry Pi:<br>
Put the Raspberry Pi into the bottom shell in the correct direction. In order to avoid installing the camera in this step, it is best to install the CSI cable corresponding to the Raspberry Pi camera**, and put the support column into the M2 from the bottom.  .5 Fix it with the Raspberry Pi, and lead the CSI cable out of the corresponding slot of the shell.
<div align=center><img width="540" height="400" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/pi1.jpg"/></div>
<br>

(2) Install the controller:<br>
Lead the controller NRF antenna out of the corresponding round hole on the bottom shell, and install it on the Raspberry Pi in the correct orientation. Note that there is only a 4P female port on the control board. The **head pin does not correspond to the Raspberry Pi head pin*  *Moreover, the pin header is outside, the correct installation should be as shown in the figure below:
<div align=center><img width="540" height="360" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/pi2.jpg"/></div>
<br>

(3) Install the upper cover: <br>
Install the upper cover on the control board in the correct direction. The main center slot should completely wrap the IMU board. After completion, the upper cover is also fixed with M2.5 screws to complete the controller assembly.
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/pi3.jpg"/></div>
<br>

## 3.2 Robot assembly 
### 3.2.1 Arm and center body assembly
(1) Assembling the robot arm requires 2 arm carbon sheets, semicircular 3D printing parts, and center body 3D printing support parts. Note that the robot arm needs to have an expansion angle of 5°. Note that the upper board card slot is L-shaped,  The lower board slot is T-shaped, and the arm is reinforced with 2mm screws. The result is shown in the following figure:
<div align=center><img width="540" height="200" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a1.jpg"/></div>
<br>
(2) Assembling the center body, the upper and lower center body carbon sheets and 4 M3*32 copper pillars are required. First, fix the copper pillars on the bottom plate.
<br>
(3) Install the arms and fix the four arms on the lower board slot. Pay attention to the 5° extension. The result is shown in the following figure:
<div align=center><img width="540" height="200" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a3.jpg"/></div>

### 3.2.2 Install battery compartment, control board and steering gear
(1) First install the battery compartment, which is installed at the bottom of the center body and the switch direction is toward the rear, the result is shown in the following figure:
<div align=center><img width="480" height="380" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a4.jpg"/></div>
<br>
(2) Install the steering gear, the steering gear is installed with the bearing down, and the outer steering gear bearing faces the inside of the body, and the inner steering gear bearing is out of the way to ensure that the legs will not be hindered by the body when moving. It is fixed on with 2mm self-locking screws  On the carbon plate, the result is shown in the figure below:
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a5.jpg"/></div>
<br>
(3) Install the control board, and install the control board above the center body (**3M glue with lock or Velcro is recommended to facilitate subsequent SD card replacement**), mainly if the controller shell is not installed, the controller needs to be 3D  The printed parts are raised to prevent short circuit between the bottom of the circuit board and the carbon plate and the estimated screws of the battery compartment. The result is shown in the following figure:
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a6.jpg"/></div>
<br>
(4) Connect the steering gear wire, wind the wire from the inside of the arm to connect the steering gear wire to the control board. Refer to the subscript for the specific connection sequence. Also note that the PWM pin is facing the inside of the body to supply 5V power. Refer to the specific pin power supply sequence  PCB IO diagram:

**Servo Wiring**
||
-------------|-------------|----------
Sch.PWM3 TIM3_3 [D_LEG] Outer servo 1|Airframe right |Sch.PWM6 TIM4_2 [D_LEG] Outer servo 2
Sch.PWM2 TIM3_2 [X_LEG] Inner steering gear 1| |Sch.PWM5 TIM4_1 [X_LEG] Inner steering gear 2
Sch.PWM1 TIM3_1 NS| |Sch.PWM4 TIM3_4 NS
**Head**||Tail
Sch.PWM7 TIM4_3 NS| |Sch.PWM10 TIM8_1 NS
Sch.PWM8 TIM4_4 [X_LEG] Inner servo 3| |Sch.PWM11 TIM8_2 [X_LEG] Inner servo 4Sch.PWM9 TIM1_1 [D_LEG] Outer Servo 3|Body Left |PSch.WM12 TIM8_3 [D_LEG] Outer Servo 4
Sch.PWM9 TIM1_1 [D_LEG] Outer Servo 3|Body Left |PSch.WM12 TIM8_3 [D_LEG] Outer Servo 4

### 3.2.3 Connect the electronic module and the upper board
(1) Weld the step-down module with the battery compartment, pay attention to the input and output relationship and the positive and negative poles. The results are shown in the following figure:
<br>
(2) Connect the step-down module through the XH2.8-8 line, and insert the step-down module into the card slot at the back of the base plate. The result is shown in the figure below:
<div align=center><img width="540" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a8.jpg"/></div>
<br>
(3) Install the upper plate, install the upper plate corresponding to the step-down module and the arm card slot, and fix the upper plate with 4 supporting copper pillars with screws. The result is shown in the following figure:
<div align=center><img width="540" height="360" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a9.jpg"/></div>


## 3.3 Leg mounting, deviation calibration and IMU sensor calibration (**very important**)
<br>**Video tutorial link：https://www.bilibili.com/video/av47485521** <br>

**Quick calibration using OLDX-Remoter**<br>

Conditions|Description
-------------|-------------
Dial 1 is on the right and the others are on the right Press and hold the joystick switch for 6 seconds | calibrate the accelerometer and gyroscope after the beep (need to stand still)
Dial 1 is on the left and the others are on the right. Press and hold the remote switch for 6 seconds | the yellow light is always on, the buzzer sounds intermittently, and the controller rotates 360° on each side for 12 seconds to complete the magnetometer calibration
Dials 1 and 2 are both on the left and the other is on the right and fluctuates vertically for many times. The remote stick | enters the steering gear calibration mode after the prompt sound, the steering gear is automatically powered (the legs need to be removed), and the buzzer prompts the steering gear number by times  , Chang the remote lever to change the selection of the steering gear, and fine-tune the steering gear deviation up and down


**note!  !  !  ：Since the IMU is not firmly connected to the control board with a flexible wire or is squeezed, SPI communication errors will occur. Therefore, when the circuit board is working normally, module.flash and module.nrf are both 1 and the original value of acceleration in mems is about  It is about [0 0 4096]. If it is not normal, please power off and reconnect the cable DEBUG to confirm.  In addition, after the circuit board is powered on, if the PX4 is turned on, the probability is more than 90%, and the probability is normal. At this time, only need to check whether the mems sensor data has an error jump!  !  **

(1) Assemble the leg support structure. First, do not connect the 3D printed parts of the sole to facilitate subsequent leg deviation installation. Assemble the gear arm attached to the steering gear with the 3D printed parts and fix them with 1.25mm screws to prevent slipping. The results are shown in the following figure.  Show:
<div align=center><img width="540" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a10.JPG"/></div>
<br>
(2) Connect the downloader to check whether the power supply of the robot power supply test system is normal, and the DEBUG program will calibrate the deviation of the leg structure: (The firmware above 1.3 supports remote control calibration, please refer to the word document)
a. First add force_dj_off_reset under vmc_demo.c file to watch and set it to 1 in DEBUG to reset the servo deviation.  <br>
b. Set vmc_all.sita_test[4] to 1, and the rudder will be powered and turned to 90° under the default deviation.  <br>c. Install the front and rear steering gear rocker arms so that they are level with the body, as shown in the figure below:
c. Install the front and rear steering gear rocker arms so that they are level with the body, as shown in the figure below:
<div align=center><img width="650" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a11.jpg"/></div>
d. Modify the deviation parameter in vmc[i].param.PWM_OFF, [0] corresponds to the angle of the outer steering gear [1] is the inner steering gear. Adjust the parameters and visually calibrate the steering gear to ensure that the steering gear is 90° absolutely vertical.  After all the angles are calibrated, fix the bearing with the supporting screws of the steering gear.  <br>
e. Set mems.Gyro_CALIBRATE to 1, and store the current calibration deviation in FLASH.  <br>
f. Reset the chip to check whether the read deviation is consistent.
<br>

(3) Calibrate the IMU sensor, place the robot horizontally in the battery compartment for sensor zero-bias calibration, set mems.Gyro_CALIBRATE and mems.Acc_CALIBRATE to 1 respectively to complete the calibration of the gyroscope and accelerometer.  <br>
**Note: When calibrating the acceleration, you will find that the posture solution is 0°, and if there is a deviation, the stability of the robot will deviate to the left or right during the movement.  Therefore, it is recommended to only set mems.Gyro_CALIBRATE to 1 when performing FLASH hold for some control parameters and servo deviation calibration.**
<br>

(4) Install the 3D printed parts of the sole. If there is a sole sensor, connect the corresponding sensor on the control board. If not, the shock-absorbing sole is used by default. Cut the 15*8mm round cushion into a semicircle and stick it to the bottom.  ,As shown below:
<div align=center><img width="480" height="360" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/leg_end.jpg"/></div>

**Note that the oblique tips of the 4 soles should face the nose when installing the soles!  !  !  **, finally complete the assembly of the whole machine:
<div align=center><img width="480" height="250" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a12.jpg"/></div>


## 3.4 Introduction of OLDX-Remote
____OLDX-Remote is a universal remote control and data monitor in the OLDX robot development platform. It can be compatible with the OLDX-FC flight control and can also be used as the remote control of a quadruped robot. In addition to using the motion sensing mode to remotely control the robot, it can also replace  The PC ground station performs real-time parameter adjustment. In addition, it sends back the running status of the SDK main state machine and sub-state machine in real time, the expected altitude and speed information and the waypoint information of the route, and can understand the current SDK operation status and the target mission of the aircraft at the next moment in real time.  This solves the problem that it is difficult to obtain the internal state of the robot in autonomous control.
<div align=center><img width="640" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/tunning.jpg"/></div>

### 3.4.1 Interface introduction
____OLDX-Remote has three types of interfaces after booting. (1) Main interface: displays common data such as robot height, posture, voltage and time; (2) PID parameter interface and PC terminal ground station correspond to display robot internal parameters (current version is temporarily useless)  (3) SDK interface: display route and autonomous task commands, state machine status.  The English abbreviations in each interface are as follows:


**Note: KEIL5 related software and chip library download：**<br>
```
link：https://pan.baidu.com/s/1UNnMhOecPHUjdAQqanoYLQ 
Extraction code：az23 
```
# 5 Donation and project follow-up development plan
____The team plans to launch a 5kg~10kg-level footed robot development chassis in the later stage, supporting RPlidar lidar navigation for SLAM algorithm verification, which can replace the current similar four-wheeled vehicle platforms on the market such as Autolabor at the same price.
 <div align=center><img width="800" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/r1.jpg"/></div>
____If you think this project is helpful to you, and for better project promotion and software and hardware updates, please donate this project through WeChat if you want!
<div align=center><img width="440" height="300" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/master/support_file/img_file/pay.png"/></div>



