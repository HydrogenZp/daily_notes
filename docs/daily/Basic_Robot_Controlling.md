# 机器人控制基础

## 闭环控制 (Closed-loop Control)

 ### 基本概念
 闭环控制通过传感器实时获取被控对象的输出反馈，计算当前输出与目标值的误差（$e = \text{目标值} - \text{实际值}$），并根据误差调整控制量。其核心是**负反馈调节**（抑制误差）与**正反馈调节**（放大特定信号），其中负反馈是控制系统中最常用的机制。

---

 ## PID控制器

 ### PID控制器介绍
 PID（比例-积分-微分）控制器是闭环控制中最经典的算法，通过线性组合误差的比例（P）、积分（I）、微分（D）项生成控制量：
$$
 u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
$$

 ### 比例、积分、微分作用
 1. **比例控制（P）**  
    - 输出与误差成正比（$u_p = K_p \cdot e$）。
    - 快速响应误差，但单独使用可能导致稳态误差（如摩擦力无法克服）。
    
 2. **积分控制（I）**  
    - 输出与误差的累积成正比（$u_i = K_i \cdot \int e \, dt$）。
    - 消除稳态误差，但可能引发超调或振荡。
    
 3. **微分控制（D）**  
    - 输出与误差变化率成正比（$u_d = K_d \cdot \frac{de}{dt}$）。
    - 预测误差趋势并抑制振荡，但对噪声敏感。

 ### 代码实现（肯定部署不到实机的）
 ```c
 #include <stdio.h
 
 typedef struct {
     float Kp;          // 比例系数
     float Ki;          // 积分系数
     float Kd;          // 微分系数
     float prev_error;  // 上一次误差
     float integral;    // 积分累加值
 } PID_Controller;
 
 // PID计算函数（参数：控制器实例，目标值，当前值，时间步长dt）
 float pid_compute(PID_Controller *pid, float target, float current, float dt) {
     float error = target - current;
     pid-integral += error * dt;
     float derivative = (error - pid-prev_error) / dt;
     float output = pid-Kp*error + pid-Ki*pid-integral + pid-Kd*derivative;
     pid-prev_error = error;
     return output;
 }
 
 int main() {
     // 示例：初始化PID控制器
     PID_Controller pid = {0.5, 0.1, 0.2, 0.0, 0.0};
     float target = 10.0;
     float current = 0.0;
     float dt = 0.1;
 
     for(int i=0; i<100; i++) {
         float control = pid_compute(&pid, target, current, dt);
         // 模拟系统响应（此处仅为示例）
         current += control * dt;
         printf("Control: %.2f, Current: %.2f\n", control, current);
     }
     return 0;
 }
 ```

 ### 调参技巧
 1. **Ziegler-Nichols法**：通过临界比例增益和振荡周期确定参数。（太jb复杂了我都没听说过，搜了才知道的）
 2. **试凑法**：  
    - 先调$K_p$至系统响应快速且无振荡；  
    - 再调$K_i$消除稳态误差；  
    - 最后用$K_d$抑制振荡。（还是得靠经验的）

---

 ## 前馈控制 (Feed Forward Control)

 ### 原理与公式修正
 前馈控制基于系统模型直接预测所需控制量，无需等待反馈。例如，对于已知的目标轨迹变化率，前馈量可表示为：
$$
 u_{ff} = K_{ff} \cdot \text{d}r(t)
$$
 其中$\text{d}r(t)$为目标轨迹的微分（如速度指令）。  
 **关键特性**：  

 - 开环控制，无法修正模型误差或外部扰动。  
 - 常与闭环控制结合（如PID+前馈），提升动态响应。

---

 ## 运动学解算（非常重要！！！）
 ### 逆运动学 (Inverse Kinematics)
 **定义**：根据底盘期望的运动状态（$v_x, v_y, \omega$）计算各轮目标转速。  
 **麦克纳姆轮逆解公式**：（别被矩阵唬住了，展开来很简单的）其中$r$为轮半径，$l$为轮组到中心距离。
$$
 \begin{bmatrix}
 \omega_1 \\
 \omega_2 \\
 \omega_3 \\
 \omega_4
 \end{bmatrix}
 = \frac{1}{r}
 \begin{bmatrix}
 1 & -1 & -l \\
 1 & 1 & l \\
 1 & 1 & -l \\
 1 & -1 & l
 \end{bmatrix}
 \begin{bmatrix}
 v_x \\
 v_y \\
 \omega
 \end{bmatrix}
$$

 

 ### 正运动学 (Forward Kinematics)
 **定义**：根据各轮速度计算机器人底盘的运动状态（线速度$v$、角速度$\omega$）。  
 **示例：麦克纳姆轮底盘**  
 假设四轮麦克纳姆轮分布角度为45°，正运动学模型为：
$$
\begin{bmatrix}
 v_x \\
 v_y \\
 \omega
 \end{bmatrix}
 = \frac{r}{4}
 \begin{bmatrix}
 1 & 1 & 1 & 1 \\
 1 & -1 & -1 & 1 \\
 -\frac{1}{l} & \frac{1}{l} & -\frac{1}{l} & \frac{1}{l}
 \end{bmatrix}
 \begin{bmatrix}
 \omega_1 \\
 \omega_2 \\
 \omega_3 \\
 \omega_4
 \end{bmatrix}
$$