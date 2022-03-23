# arduino-Infrared-sensor-door
基于arduino的自动感应红外门

1.目的和意义（不少于200字）
目的：本项目(自动红外门控制器)的目的是利用红外感应，感知是否有人或恒温动物经过，若有人经过，则使门自动打开。达到智能化控制的现象。
意义：在超级市场、公共建筑、银行、医院入口，经常使用自动门控制系统。使用自动门，可以节约空调能源、降低噪音、防风、防尘，同时可以使出口显得庄重高档，因此应用非常广泛。自动平移门最常见的形式是自动门及门内外的开关，当人走进自动门时，感应开关感应到人的存在，给控制器一个开门的信号，控制器通过驱动装置将门打开，当人通过门之后，在将关闭。由于自动平移门在通电后可以实现无人管理，既方便又提高了建筑档次，于是迅速在国内外建筑市场上得到了广泛应用。

关于热释电红外传感器HC-SR501模块
HC-SR505 小型人体感应模块是基于红外线技术的自动控制产品, 
灵敏度高，可靠性强，超小体积，超低电压工作模式。广泛应用于各 类自动感应电器设备,尤其是干电池供电的自动控制产品。
热释电红外传感器是根据菲涅耳原理制成，把红外光线分成可见区和盲区，同时又有聚焦的作用，使热释电人体红外传感器 （PIR） 灵敏度大大增加。菲涅耳透镜折射式和反射式两种形式，其作用一是聚焦作用，将热释的红外信号折射（反射）在PIR上；二是将检测区内分为若干个明区和暗区，使进入检测区的移动物体能以温度变化的形式在PIR上产生变化热释红外信号，这样PIR就能产生变化电信号。如果我们在热电元件接上适当的电阻，当元件受热时，电阻上就有电流流过，在两端得到电压信号。

电路连接：

![image](https://user-images.githubusercontent.com/57294382/159699448-d6b4f23e-b88a-4848-ba6b-5abe3a68c557.png)

流程图：

 ![红外门](https://user-images.githubusercontent.com/57294382/159698981-b17ed186-1678-4d52-9dc4-aa60c10f07b9.jpg)

![3s判定](https://user-images.githubusercontent.com/57294382/159699046-0310eb1c-4a91-428f-b314-9b43e9e17f36.png)

最后没有用小电机，经过老师建议后用了比较大的42电机，42连线图可以在代码的首段注释中找到
