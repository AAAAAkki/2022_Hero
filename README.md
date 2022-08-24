# Hero-2022

[TOC]

## New functions and features

> 本节将列出英雄控制方面的新功能和特征
> 所有内容将在下一节按序描述细节后，再简述代码逻辑
1. 双C板上下通信
2. 绝对云台控制模式下，对云台位置微调
3. 云台添加瞄准镜（2006电机），对瞄准镜的开环控制
4. 放弃键盘控制时底盘静止（在云台微调时和开镜状态下）

**没怎么用到的小功能**

- 激光舵机控制
- C板KEY键开关蜂鸣器

## Operating Details

> 本节将说明部分功能的细节及实现逻辑，以帮助后续维护和开发

### 上下C板通信
受限于导电滑环的线路数，位于底盘的设备无法将所有信号线（CAN线与串口线）通过导电滑环与位于云台的C板通信，因此英雄采用上下双C板控制，即云台和底盘各置一块C板。其中云台C板负责主要的控制和计算，起到类似于“大脑”的功能。而底盘C板的定位类似“脊柱”，主要负责底盘设备的通信中转（包括4个底盘电机，yaw轴6020，拨弹轮和裁判系统），同时超级电容控制板直接由下C板单独控制。

<img src="https://s2.loli.net/2022/08/15/wHbGNtfKV3JIS8Q.png" style="zoom:80%;" />



这份英雄代码对应的硬件连线如图所示。上下C板之间的通信交换了：自下向上的裁判系统数据，自上向下的底盘速度数据和UI绘制所需数据。因为YAW电机和拨弹轮并入CAN2回路，不需要底盘C盘单独接收再转发。

#### 代码简述

##### 底盘控制

此部分涉及的内容主要位于云台和底盘各自文件夹下的`CAN_receive.c`和`chassis.c` 前者包含了所有的CAN通信打包拆包的约定，后者包含了通过上C板控制底盘运动的逻辑。下面就底盘运动逻辑进行简述，因为这部分代码和原始代码有较大出入。
原始控制流程大致为：**接收遥控器数据-->设置状态机和英雄速度**-->速度解算-->解算数据计算PID后发送给电机-->接收电机反馈数据，用于下一次PID。
流程的前两步需要与遥控器接收数据和云台状态，只能在上C板中运行。而电机的控制需要PID计算，在下C板运行更实际。因此，上板发送的底盘控制数据是包含`vx_set,vy_set,wz_set`以及`chassis_mode`的设定值，解算和控制全部交给下C板。 p.s. 下板数据结构

##### 裁判系统
裁判系统数据由下C板发送给上C板。下C板运行裁判系统串口任务中的数据解算函数`referee_data_solve` 得到数据后即发送给上板。上板创建了新的结构体`chassis_data_receive`来接收下板发送数据。
##### UI数据
由上C板发送给下C板。实现函数为`send_gimbal_motor_state`。发送数据冗杂，由下板的结构体接收并直接在UI任务中调用。

p.s. 下板数据结构

### 云台位置微调

当前键位设置为：*c f v b*。原理即是在云台绝对角度控制流程的最后加上对set值的微小加减。

#### 代码简述

实现函数：`key_micro_control`

简单的借用标志位进行锁定。解锁时功能才能触发，锁定时对标志位做增/减操作，一定周期后解锁。大多数增加的按键都使用这种方法，避免按一次键触发多次的“鬼键”。~~（这段解释一定是多余的吧）~~

### 瞄准镜

瞄准镜的2006接入上C板的CAN1回路中，所以和原代码比CAN发送函数增加一个参数。瞄准镜的电机控制有两套控制方案，分别为开环控制和~~半~~闭环控制。为了克服可能存在的虚位问题，目前采用开环控制。

控制过程为：改变瞄准镜状态时，赋予一个绝对值递减的电流值驱动，其余时候赋予一个小电流锁住瞄准镜的位置。

~~虽然没有用闭环，不过还是简单讲一下，最后的实现效果未知~~闭环指在锁定瞄准镜位置时使用编码器闭环，且要用编码器闭环而非速度闭环才能完全锁死。

#### 代码简述



### 静止底盘



#### 代码简述




## Todo List


自瞄射击数据通信--计算数据返回控制云台

弹速自适应逻辑优化

新版超级电容控制（抄步兵作业）

英雄视觉

***

#### To figure out

左右偏移因素？

Q1 发射时云台抖动曲线是否证明弹丸接触左右摩擦轮有先后
Q2 第二轮射击（20发以后）左右散布会增大？

***
