Cosmic Ray detection with ESP32S3， running freertos， ver.0.1

MPL2.0许可协议，修改不可闭源，但新增代码协议不必延续MPL协议


1.1. git仓库会忽略build与Archives与Binaries目录下的所有文件，因此每次从远程仓库同步后需要重新build。

1.2. 协作开发方式：每次实现新功能前，从远程仓库拉取（pull）最新版本代码到本地仓库，再新建一个分支（branch）。本地仓库多次
commit后，编译测试实现了期望的功能，把本地仓库push到新建分支上，同时在push时提交一份说明。之后再提交一个pull request，
申请将新分支与main分支合并，经过其他成员测试审核后通过合并请求，正式合并。

1.3. commit is cheap，commit 仅提交代码到本地仓库，因此为了版本管理方便起见应该多commit。但每次commit应该简单描述
commit的改动和期望实现的目的，以便维护一个完整的change log。

1.4. 提交bug issue与featrue issue请按照仓库内issue template格式。

git使用细节参考（https://zhuanlan.zhihu.com/p/51199833 ）

2.1. FreeRTOS直接使用ESP-IDF组件，因此没有包含在工程目录下，需要正确配置ESP-IDF。

2.2. ESP-IDF自带组件的配置全部通过工程下sdkconfig完成，因此如果对sdkconfig有改动，也应当新建branch进行测试。



3.1. 自定义的变量名称应当使用英文或英文缩写，尽可能简短而清晰地说明这个变量的功能。具体命名规范为大驼峰命名法，
（参考 https://blog.csdn.net/weixin_43758823/article/details/84888470 ），每个逻辑断点的单词的首字母大写，不需要
下划线。示例：需要设置一个存储科学数据的缓存，命名为SCIBuf。

3.2. main.c内定义的函数仅包含任务函数、中断服务子程。自定义子函数应当在BSP文件夹下新开.c和.h文件。

3.3. 在bsp.h内include BSP文件夹内所有其它.h文件，这样只需要在.c文件内 include一次bsp.h，就能够使用BSP内所有自定义函数。

3.4. 写.h文件时注意使用#ifndef #define #endif语句，来避免.h文件重复include。

3.5. 所有全局变量都在main.c内定义，并在bsp.h内通过extern关键字声明，以此避免重复声明。如果全局变量使用了自定义结构体，
结构体也在bsp.h内声明。如果有.c文件需要用到全局变量，就include bsp.h。

示例：

在bsp.h内声明SCIBuf结构体
typedef struct SCIPkg {
    uint8_t                 Head[3];
    SCIDataType             Event;
    SCIDataShortType        EventShort[43];
    uint8_t                 EffCnt[4]; 
    uint8_t                 LostCnt[4];
    uint8_t                 Tail[3];
    uint8_t                 CRC[2];
}SCIPkgType;

在bsp.h内通过extern关键字声明SCIBuf全局变量

extern          SCIPkgType      SCIBuf[2][8];

在main.c内定义SCIBuf全局变量

SCIPkgType      SCIBuf[2][8]     = {0};

3.6. 作者信息与代码功能在代码内通过注释体现，示例：

/*
High flux mode global var   (edit by LLH 2024.11.21)
*/
