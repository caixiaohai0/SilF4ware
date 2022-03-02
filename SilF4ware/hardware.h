//注：引脚和端口定义位于main.h 中
//用于 LVC 的东西
#define ENABLE_ADC

// 如果引脚为低电平时 LED 亮起，请使用此选项
#define LED_INVERT

// 在 DMA 或基于 NOP 的延迟版本之间进行选择
#define DSHOT_DMA_BIDIR // 需要RPM_FILTER，最大 4k 环路频率
//#define DSHOT_DMA_DRIVER // 常规 Dshot，消耗较少的周期，适用于 8k 循环频率
//#define DSHOT_DRIVER // 延迟版本

#if defined FC_BOARD_OMNIBUS || defined FC_BOARD_F4XSD
	#define RX_SOFTSPI_4WIRE
	#define MPU_HARDSPI 1
	#define OSD_HARDSPI 3
#elif defined FC_BOARD_NOXE
	#define RX_HARDSPI 1
	#define MPU_HARDSPI 2
	#define OSD_HARDSPI 2
#elif defined FC_BOARD_NOXE_V1
	#define RX_HARDSPI 2
	#define MPU_HARDSPI 1
	#define OSD_HARDSPI 2
#endif

// RX_HARDSPI 或 4 线或 3 线RX_SOFTSPI（用于与无线电模块通信）
//#define RX_HARDSPI 1 // only SPI1 and SPI2 implemented
//#define RX_SOFTSPI_4WIRE // used for XN297 and NRF24
//#define RX_SOFTSPI_3WIRE // used for XN297L; SPI_RX_MOSI is used for data output and input

// MPU_HARDSPI或MPU_SOFTSPI（用于与陀螺仪芯片通信）
//#define MPU_HARDSPI 1 // 仅实现了 SPI1 和 SPI2
//#define MPU_SOFTSPI
