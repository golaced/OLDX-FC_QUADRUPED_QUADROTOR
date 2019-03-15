
# 1 OLDX-MocoMoco四足机器人开发平台项目  
<div align=center><img width="600" height="130" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/logo.JPG"/></div>
<div align=center><img width="440" height="280" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/fc.jpg"/></div>
  OLDX-MocoMoco四足机器人（OLDX-MocoMoco Quadruped）是目前国内首个真正意义上的开源足式机器人二次开发平台，团队已经开发了一套完整的免费开源
  项目OLDX-FC，随着近年来波士顿动力公司不断推出的足式机器人引起了国内外许多爱好者的关注，也出现了许多电子发烧友自己开发的足式机器人，但
  不同于四轴飞行器仅采用无刷电机和机架就能进行开发，而足式机器人本身结构复杂特别是伺服驱动部分可以独立作为一个学科进行研究，因此造成了
  虽然有许多不错的足式机器人项目但是由于项目成本或者编程语言不为国内环境所熟悉的原因而阻碍了它们的推广或开源。团队4年前就开始研发的六
  足机器人具有丰富的机器人，通过将其与所开发的飞控项目二次结合推出了MocoMoco四足机器人开发平台，囊括机器人结构、步态算法和SLAM无人驾驶
  等多个足式机器人核心技术内容。项目遵循GPL协议，能对DEMO内相关源码进行修改和二次开发。<br><br><br>

**-如果该项目对您有帮助请 Star 我们的项目-**<br>
**-如果您愿意分享对该项目的优化和改进请联系golaced@163.com或加入我们的QQ群567423074，加速开源项目的进度-**<br>

	
# 2 MocoMoco四足机器人平台介绍

 <div align=center><img width="240" height="300" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/pay.png"/></div>

MocoMoco四足机器人是一个8自由度足式机器人其借鉴了GhostRobtic推出的Minitature机器人，为降低伺服驱动成本将其采用的无刷电机替换为高性能
舵机。控制器方面采用了目前国内开发群体较多的STM32和树莓派，同时该硬件架构将在后续机器人项目中沿用将首先移植OLDX-FC飞控项目实现真正的
一板多用的目的。电路板硬件方面采用了一体化集成设计采用最小推出的树莓派A3+具有小体积高性能的特点能为后续图像、激光雷达SLAM提供开发可能，
控制板采用STM32F4作为处理器，板载3轴加速度计、3轴陀螺仪、3轴磁力计和气压计采用内减震设计，电路安装孔能完美与树莓派安装同时提供3D打印外壳。
控制板采用外部可更换供电模块供电具有12路PWM输出和4路AD采样，并外扩4路串口兼容正点原子推出的无线调试其方面机器人不同调试。

**控制器硬件参数：**

项目|参数
-------------|-------------
处理器|STM32F405RGT6
处理器性能|32Bit ARM Cortex-M4 168MH
陀螺仪 加速度计|LSM6DS33 
磁力计|LIS3MDL
气压计|MS5611
预留接口|GPS-1 串口-4 AD采样-4 着地开关输入-4
PWM 输出通道| 12通道输出 
供电|5V输入 舵机外部供电  AD传感器3.3/5V供电选择
图像处理器|树莓派A3+  1.4G 4核  带独立供电开关
遥控方式|2.4G射频 SBUS航模遥控
地面站|QGround   匿名地面站(需要额外OLDX-REMOTE监视器)

**机器人参数：**

项目|参数
-------------|-------------
足式机器人类型|8自由度并联机器人
尺寸|30cm * 20cm *10cm  
全腿长|8cm
重量|600g
供电|7.4V  18650-2 带开关 
步态支持|Tort步态  Fly-Trot步态  波动步态
最大移动速度|0.4m/s
最大转向速度|30度/s
步态周期|>0.35 s
舵机性能|Kpower 12g金属舵机-8个  6V-60度/0.035s  9kg扭矩
足底传感器|薄膜压力/微动开关(默认无传感器)
控制模式|遥控模式  姿态平衡模式(有/无着地传感器)  驾驶模式  

# 3 DEMO测试
## 3.1 机器人组装


## 3.2 腿部偏差校准与传感器校准


## 3.3 移动测试



## 3.4 高度轨迹和姿态轨迹跟踪测试


## 3.5 DEMO程序开发



# 4 SDK开发


# 5 捐赠与项目后续开发计划
如果您觉得该项目对您有帮助，也为了更好的项目推进和软硬件更新，如果愿意请通过微信捐赠该项目！
<div align=center><img width="240" height="300" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/pay.png"/></div>





