_本仿真文件对应的任务为***空对地导弹***，控制方式采用***基于图像识别的PN控制***，控制对象为鸭式布局的轻型导弹模型。
## 快速上手
- 在Inital.m文件中配置参数并运行将变量加载到工作区
- 使用cam_xyz_missile_model进行仿真，该文件目标线与基准线的夹角由图像识别得出
- (备选)使用xyz_missile_model进行仿真，该文件目标线与基准线的夹角由已知导弹坐标与目标点坐标解算得出

## 一、导弹模型建立
_本仿真中使用的坐标系与符号若无特殊说明，均与《导弹飞行力学》(钱杏芳)统一_
### 1.1受力模块
#### 1.1.1总空气动力(速度坐标系)
- 该函数用于计算导弹在**速度坐标系**下所受的空气动力，包括阻力（$F_{x3}$）、升力（$F_{y3}$）和侧向力（$F_{z3}$）。  
- **速度坐标系定义**：以导弹质心为原点，X轴与速度矢量同向，Y轴在导弹纵向对称面内垂直X轴向上，Z轴垂直于XY平面（右手螺旋法则确定）。

空气动力的通用计算公式为：
 $$F = \frac{1}{2} \rho V^2 S C $$
  其中：  
- $\frac{1}{2} \rho V^2$ 为**动压**（气流动能的量度），$\rho$ 为空气密度  
- $S$ 为参考面积  
- $C$ 为无量纲的气动力系数（与导弹姿态相关）
  
##### 升力公式推导：
1. **升力系数**（小攻角假设下呈线性关系）：  
   $$ C_L = C_{L\alpha} \cdot \alpha $$  
   其中 $C_{L\alpha} = 4.0 \, (1/\text{rad})$ 为升力线斜率（经验值，反映升力随攻角的变化率）

2. **升力公式**（代入通用公式）：  
   $$ Fy = \frac{1}{2} \rho V^2 S C_L = \frac{1}{2} \rho V^2 S C_{L\alpha} \alpha $$

##### 阻力公式推导：
1. **零升阻力系数**：  
   $$ C_{D0} = 0.05 \, (\text{经验值，与导弹外形相关}) $$

2. **诱导阻力系数**（与升力系数平方成正比，与展弦比成反比）：  
   $$ C_{Di} = \frac{C_L^2}{\pi \cdot AR \cdot e} $$  
   其中 $e = 0.7$ 为Oswald效率因子（反映机翼平面形状对诱导阻力的影响）

3. **总阻力系数**：  
   $$ C_D = C_{D0} + C_{Di} $$

4. **阻力公式**（代入通用公式）：  
   $$ Fx = \frac{1}{2} \rho V^2 S C_D = \frac{1}{2} \rho V^2 S \left( C_{D0} + \frac{C_L^2}{\pi \cdot AR \cdot e} \right) $$ 

##### 侧向力公式推导： 
1. **侧力系数**（小侧滑角假设下呈线性关系）：  
   $$ C_Y = C_{Y\beta} \cdot \beta $$  
   其中 $C_{Y\beta} = -3.5 \, (1/\text{rad})$ 为侧力线斜率（负号表示侧向力方向与侧滑角相反）

2. **侧向力公式**（代入通用公式）：  
   $$ Fz = \frac{1}{2} \rho V^2 S C_Y = \frac{1}{2} \rho V^2 S C_{Y\beta} \beta $$  

#### 1.1.2旋转力矩(弹体坐标系)
_正常布局与鸭式布局有所差异，本次仿真针对鸭式布局，计算中认为低空环境，空气密度与动态压参数如下：_
- **空气密度**
（50m高度近似值）
$$ \rho = 1.225 \, \text{kg/m}^3 $$

- **动态压**
$$ Q = 0.5 \cdot \rho \cdot V^2 $$
其中 $V$ 为导弹飞行速度（m/s）

##### 1.1.2.1俯仰力矩($M_z$)计算

###### 一、核心计算公式
**1.1 俯仰力矩通用表达式**
$$ Mz = Q \cdot S \cdot l_{\text{ref}} \cdot C_m $$
其中：
- $Q$：动态压（Pa）
- $S$：参考面积（m²）
- $l_{\text{ref}}$：弹体参考长度（m）
- $C_m$：无量纲俯仰力矩系数


###### 二、关键参数计算公式

**2.1 无量纲俯仰角速度**
$$ \hat{q} = 
\begin{cases} 
\frac{q \cdot l_{\text{ref}}}{2 \cdot V} & \text{当} \, V \geq 1 \, \text{m/s} \\
0 & \text{当} \, V < 1 \, \text{m/s} 
\end{cases}
$$
其中 $q$ 为俯仰角速度（rad/s）


###### 三、俯仰力矩系数（$C_m$）
鸭式布局专用力矩系数公式：
$$ C_m = C_{m\alpha} \cdot \alpha + C_{m\delta} \cdot \delta_e + C_{mq} \cdot \hat{q} + C_{m\text{duck-couple}} \cdot \alpha \cdot \delta_e $$

**3.1 各气动系数说明**
| 系数 | 推荐值 | 物理意义 |
|------|--------|----------|
| $C_{m\alpha}$ | -0.7 (1/rad) | 攻角导数（推荐范围：-0.6~-0.8） |
| $C_{m\delta}$ | 0.5 (1/rad) | 鸭翼舵偏导数（推荐范围：0.4~0.6） |
| $C_{mq}$ | -2.3 (1/rad) | 俯仰角速度阻尼（推荐范围：-2.0~-2.5） |
| $C_{m\text{duck-couple}}$ | 0.2 (1/rad²) | 鸭翼-攻角耦合项 |


###### 四、最终俯仰力矩完整公式
$$ Mz = 0.5 \cdot \rho \cdot V^2 \cdot S \cdot l_{\text{ref}} \cdot \left( C_{m\alpha} \cdot \alpha + C_{m\delta} \cdot \delta_e + C_{mq} \cdot \hat{q} + C_{m\text{duck-couple}} \cdot \alpha \cdot \delta_e \right) $$

变量说明：
- $\alpha$：攻角（rad）
- $\delta_e$：鸭式升降舵偏角（rad，正为上偏）
- 其他参数同前文定义
 
##### 1.1.2.2偏航力矩($M_y$)计算
###### 一、核心计算公式
**1.1 偏航力矩通用表达式**
$$ My = Q \cdot S \cdot l_{\text{ref}} \cdot C_n $$
其中：
- $Q$：动态压（Pa）
- $S$：参考面积（m²）
- $l_{\text{ref}}$：弹体参考长度（m）
- $C_n$：无量纲偏航力矩系数



###### 二、关键参数计算公式
**2.1 无量纲偏航角速度**
$$ \hat{r} = 
\begin{cases} 
\frac{r \cdot l_{\text{ref}}}{2 \cdot V} & \text{当} \, V \geq 1 \, \text{m/s} \\
0 & \text{当} \, V < 1 \, \text{m/s} 
\end{cases}
$$
其中 $r$ 为偏航角速度（rad/s）


###### 三、偏航力矩系数（$C_n$）
鸭式布局力矩系数公式：
$$ C_n = C_{n\beta} \cdot \beta + C_{n\delta} \cdot \delta_r + C_{nr} \cdot \hat{r} + C_{n\text{duck-couple}} \cdot \beta \cdot \delta_r $$

**3.1 各气动系数说明**
| 系数 | 推荐值 | 物理意义 |
|------|--------|----------|
| $C_{n\beta}$ | -0.7 (1/rad) | 侧滑角导数（推荐范围：-0.6~-0.8） |
| $C_{n\delta}$ | 0.5 (1/rad) | 方向舵偏导数（推荐范围：0.4~0.6） |
| $C_{nr}$ | -2.3 (1/rad) | 偏航角速度阻尼（推荐范围：-2.0~-2.5） |
| $C_{n\text{duck-couple}}$ | 0.2 (1/rad²) | 鸭翼-侧滑角耦合项 |



###### 四、最终俯仰力矩完整公式
$$ My = 0.5 \cdot \rho \cdot V^2 \cdot S \cdot l_{\text{ref}} \cdot \left( C_{n\beta} \cdot \beta + C_{n\delta} \cdot \delta_r + C_{nr} \cdot \hat{r} + C_{n\text{duck-couple}} \cdot \beta \cdot \delta_r \right) $$

- 变量说明：$\beta$为侧滑角（rad），$\delta_r$为鸭式方向舵偏角（rad，正为右偏）
  
##### 1.1.2.3滚转力矩($M_x$)计算
###### 一、核心计算公式
**1.1 滚转力矩通用表达式**
$$ Mx = Q \cdot S \cdot b \cdot C_l $$
其中：
- $Q$：动态压（Pa）
- $S$：参考面积（m²）
- $b$：翼展（m）
- $C_l$：无量纲滚动力矩系数

###### 二、滚转力矩系数（$C_l$）
鸭式布局力矩系数公式：
$$ C_l = C_{lp} \cdot \hat{p} + C_{l\delta} \cdot \delta_a + C_{l\alpha\beta} \cdot \alpha \cdot \beta + C_{l\text{duck-couple}} \cdot \delta_a \cdot (\alpha^2 + \beta^2) $$

**3.1 各气动系数说明**
| 系数 | 推荐值 | 物理意义 |
|------|--------|----------|
| $C_{lp}$ | -0.6 (1/rad) | 滚转角速度阻尼（推荐范围：-0.5~-0.7） |
| $C_{l\delta}$ | 0.3 (1/rad) | 副翼偏导数（推荐范围：0.2~0.4） |
| $C_{l\alpha\beta}$ | 0.7 | 攻角-侧滑角耦合项 |
| $C_{l\text{duck-couple}}$ | 0.1 | 鸭翼滚转非线性耦合项 |

###### 三、最终俯仰力矩完整公式
$$ Mx = 0.5 \cdot \rho \cdot V^2 \cdot S \cdot b \cdot \left( C_{lp} \cdot \hat{p} + C_{l\delta} \cdot \delta_a + C_{l\alpha\beta} \cdot \alpha \cdot \beta + C_{l\text{duck-couple}} \cdot \delta_a \cdot (\alpha^2 + \beta^2) \right) $$

- 变量说明：$\alpha$为攻角（rad），$\beta$为侧滑角（rad），$\delta_a$为鸭式副翼偏角（rad，正为右鸭翼上偏）

### 1.2导弹运动方程组
_本模型使用的运动方程组为简化方程组，与《导弹飞行力学》(钱杏芳)一致_
#### 1.2.1动力学方程组
#### 1.2.2运动学方程组
#### 1.2.3几何关系方程组
## 二、目标识别模块
_本模块模拟了实际任务中摄像头与视觉识别算法(如YOLO)功能，返回目标物在图像中的像素坐标，参考https://blog.csdn.net/chentravelling/article/details/53558096_
摄像机参数说明：
|参数名|默认值|物理意义|单位|
|------|------|------|------|
| f|18|焦距(光学中心到感光元件之间的距离)|mm|
|size|0.5*254|图像传感器对角线长度|mm|
|u_max|1920|横向像素点数量|个|
|v_max|1080|纵向像素点数量|个|

坐标系说明：
- 世界坐标系：与上文一致
- 相机坐标系：与弹体坐标系固连
- 图像坐标系：二维坐标系，为减小理解难度，设定为YOZ平面，两个轴方向与弹体坐标系两轴指向一致，此坐标系下单位为mm
- 像素坐标系：在图像坐标系上将原点设为图像左上角，横向向右为u,纵向向下为v(与图像坐标系定义一致)，单位为像素

###  2.1空间坐标系转换为相机坐标系
从空间坐标系转换到相机坐标系需要平移、旋转两步操作
#### 2.1.1平移操作
假设摄像机在世界坐标系下坐标为$\overrightarrow{cam}$,目标物在世界坐标系下坐标为$\overrightarrow{target}$,则在世界坐标系下目标物相对于摄像机的坐标为($\overrightarrow{target}$-$\overrightarrow{cam}$),平移实际上是将坐标系原点移至摄像机处，方便后续旋转操作。
#### 2.1.1旋转操作
旋转操作将目标点在世界坐标系下相对于摄像机坐标转化为相机坐标系下目标点坐标。因相机与导弹固连，所以旋转矩阵同导弹模型，顺序为俯仰变换、偏航变换、滚转变换。如下：
$$
R_z(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$ 
$$
R_y(\psi) = \begin{bmatrix}
\cos\psi & 0 & \sin\psi \\
0 & 1 & 0 \\
-\sin\psi & 0 & \cos\psi
\end{bmatrix}
$$
$$
R_x(\gamma) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\gamma & -\sin\gamma \\
0 & \sin\gamma & \cos\gamma
\end{bmatrix}
$$
整体旋转变换为
$$R(\gamma,\psi,\theta)=R_x(\gamma)R_y(\psi)R_z(\theta)$$

整体公式如下
$$
\overrightarrow{target_c}=\begin{bmatrix}
x_c \\
y_c \\
z_c
\end{bmatrix}=R(\gamma,\psi,\theta)*(\begin{bmatrix}
x_{target} \\
y_{target} \\
z_{target}
\end{bmatrix}-\begin{bmatrix}
x_{cam} \\
y_{cam} \\
z_{cam}
\end{bmatrix})
$$

###  2.2相机坐标系转化为图像坐标系
![图片错误](1.jpg)
由图可知存在以下几何关系
$$\frac{AB}{OC}=\frac{AO_c}{OO_C}=\frac{PB}{PC}=\frac{z_c}{z}=\frac{x_c}{f}=\frac{y_c}{y}$$
式中$x_c$、$y_c$、$z_c$为目标点在相机坐标系下坐标，$z$、$y$为图片坐标系下坐标，$f$为焦距
###  2.3图像坐标系转化为像素坐标系
首先求取每一个像素点代表多少mm($dx、dy$),当横向像素点大小与纵向像素点大小相同时有以下公式：
$$dx=dy=\frac{size}{\sqrt{u_max^2+v_max^2}}$$
最后平移原点至左上方并变换y与v的方向即可
![图片错误](2.jpg)
公式如下
$$
\begin{bmatrix}
u \\
v
\end{bmatrix}=\begin{bmatrix}
\frac{x}{dx} \\
-\frac{y}{dy}
\end{bmatrix}+\begin{bmatrix}
\frac{u_{max}}{2} \\
\frac{v_{max}}{2}
\end{bmatrix}
$$
## 三、比例导引控制器设计
_此模块接受视觉模块返回的目标点在像素坐标系中的位置信息，结合相机参数以及位姿信息解算出目标线与基准线之间的夹角(参考角)，基准线为X轴_
定义以下参数
- $q_1$、$q_2$表示垂直面(XOY)、水平面下的参考角(XOZ)，注意区分正负，正方向统一定义为绕轴正方向旋转为正，也就是在垂直(XOY)中X轴为$0^。$y轴为$90^。$。在水平面(XOZ)中x轴为$0^。$z轴为$-90^。$
- $q_1'$、$q_2'$表示在相机(弹体)坐标系下求得的参考角，基准线为$x_3$轴
### 3.1求解参考角
由[2.2](#22相机坐标系转化为图像坐标系)可知目标线可表示为$
\begin{bmatrix}
x_c \\
y_c\\
z_c
\end{bmatrix}*k=\begin{bmatrix}
1 \\
\frac{y_c}{x_c}\\
\frac{z_c}{x_c}
\end{bmatrix}*k'=\begin{bmatrix}
1 \\
tan(q_1')\\
tan(q_2')
\end{bmatrix}*k'
$
对该目标线做逆旋转变换，使其变为在空间坐标系下的直线表示，公式如下：
$$\begin{bmatrix}
1 \\
tan(q_1)\\
tan(q_2)
\end{bmatrix}*k''=R^{-1}*\begin{bmatrix}
1 \\
tan(q_1')\\
tan(q_2')
\end{bmatrix}*k'=R^T*\begin{bmatrix}
1 \\
tan(q_1')\\
tan(q_2')
\end{bmatrix}*k'
$$
$q_1、q_2$得解
### 3.2控制器设计
 _采用传统PN控制，即法向负载=$K*\frac{dq}{dt}$，方向舵与升降舵由PN控制器控制，副翼由PID控制器控制，期望为0_



