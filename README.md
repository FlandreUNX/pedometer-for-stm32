# pedometer for stm32

使用stm32f401re，X-NUCLEO-IKS01A1，X-NUCLEO-IDB04A1
实时计步，可通过蓝牙传递数据

##编译
使用keil uVision 5.23 版本，上述模块的Drivers请自行下载，并在option for target设置好路径

##算法
算法是[利用 3 轴数字加速度计 实现功能全面的计步器 设计](http://www.analog.com/cn/analog-dialogue/articles/pedometer-design-3-axis-digital-acceler.html)的实现，核心部分在step()和newStep()函数中，注意在userProcess函数结尾的delay决定了采样频率