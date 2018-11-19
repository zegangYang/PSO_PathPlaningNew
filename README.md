

### PSO_PathPlaning

This open source project is a matlab GUI project,is a Robot Path Planing Demo use Particle Swarm Optimization(PSO) algorithm

### Usage

- **Open** and **run** pathPlaning.m

- **Click** button '添加圆' ->**left click and drag** cursor to generate a circle data set->Click button '结束添加'->Click button '选择起始点'->**Left click** and select start-point and end-point->Click button '规划路径'->end

  ![](https://git-resources-1258054708.cos.ap-chengdu.myqcloud.com/pathplaning.png)

### More Usage

- Get result as Bezier curve,result *(x11,y11)* and *(x21,y21)* are Bezier control points,*(x0,y0)* and *(x1,y1)* also.Show as image:

![](https://git-resources-1258054708.cos.ap-chengdu.myqcloud.com/bezier_control_points_roughly.png)

Or can get these result from code below:

```matlab
function pushbutton3_Callback(...)
...
h = waitbar(0,'Please wait...');
while Iter_current < Iter_max
    ...
end
close(h);

x11 = G_best(1)*100; x21 = G_best(3)*100;
y11 = G_best(2)*100; y21 = G_best(4)*100;
x0 = P0(1)*100;
y0 = P0(2)*100;
x1 = P1(1)*100;
y1 = P1(2)*100;
tt = 0:1/79:1;		%steps
x_t1 = (-x0+3.*x11-3.*x21+x1).*tt.^3+(3.*x0-6.*x11+3.*x21).*tt.^2+(-3.*x0+3.*x11).*tt+x0;
y_t1 = (-y0+3.*y11-3.*y21+y1).*tt.^3+(3.*y0-6.*y11+3.*y21).*tt.^2+(-3.*y0+3.*y11).*tt+y0;
%% 绘制路线数据和场地信息数据
plot(x_t1,y_t1,'.r')    %  路线数据
hold on
```

### Help

- **Q**:Why result multiply by 100?

  **A**:

TODO: 

<img src='https://g.gravizo.com/g?
 digraph systimer
{
  edge [fontname="Helvetica",fontsize="10",labelfontname="Helvetica",labelfontsize="10"];
  node [fontname="Helvetica",fontsize="10",shape=record];
  rankdir="LR";
  Node0 [label="HAL::ISysTimer(Abstract class)",height=0.2,width=0.4,color="black", fillcolor="grey75", style="filled", fontcolor="black"];
  Node0 -> Node1 [dir="back",color="midnightblue",fontsize="10",style="solid",fontname="Helvetica"];
  Node1 [label="enabotLinux::LSysTimer",height=0.2,width=0.4,color="black", fillcolor="white", style="filled",URL="$classenabot_linux_1_1_l_sys_timer.html"];
  Node0 -> Node2 [dir="back",color="midnightblue",fontsize="10",style="solid",fontname="Helvetica"];
  Node2 [label="enabotQt::QSysTimer",height=0.2,width=0.4,color="black", fillcolor="white", style="filled",URL="$classenabot_qt_1_1_q_sys_timer.html"];
  Node0 -> Node3 [dir="back",color="midnightblue",fontsize="10",style="solid",fontname="Helvetica"];
  Node3 [label="STM32F1::F1SysTimer",height=0.2,width=0.4,color="black", fillcolor="white", style="filled",URL="$class_s_t_m32_f1_1_1_f1_sys_timer.html"];
  Node0 -> Node4 [dir="back",color="midnightblue",fontsize="10",style="solid",fontname="Helvetica"];
  Node4 [label="STM32F4::F4SysTimer",height=0.2,width=0.4,color="black", fillcolor="white", style="filled",URL="$class_s_t_m32_f4_1_1_f4_sys_timer.html"];
  Node0 -> Node5 [dir="back",color="midnightblue",fontsize="10",style="solid",fontname="Helvetica"];
  Node5 [label="STM32H7::H7SysTimer",height=0.2,width=0.4,color="black", fillcolor="white", style="filled",URL="$classstm32h7_1_1_h7_sys_timer.html"];
'/>