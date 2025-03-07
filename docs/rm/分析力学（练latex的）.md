# 分析力学

## 约束
![](../imgs/simple_pendulum.png)

如单摆，建立平面直角坐标系，对小球分析，可以得到下列方程组
$$
\left\{\begin{matrix}
-T\frac{x}{\sqrt{x^2+y^2}}=m\ddot x \\  
mg-T\frac{x}{\sqrt{x^2+y^2}}=m\ddot y \\
\end{matrix}\right.
$$
其中$ x^2+y^2=l^2$，**约束力**为$ T $

$ T $使得小球在一个圆上运动，这种约束称为**几何约束**，$ x^2+y^2=l^2 $为约束方程

## 广义坐标
仅考虑单摆运动，角度比较小时对上面方程利用小角近似$\text{sin}\theta \approx \theta$，可得到
$$
\ddot{\theta}+\frac{g}{l}\cdot\dot{\theta}=0
$$
**广义坐标**不指定特定的坐标，只求简洁描述物理系统，包括极坐标等

## 虚位移

**虚位移**是指符合约束条件的无穷小位移，记为$\partial \vec r$

## 
