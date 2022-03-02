#include "defines.h"

// acro 模式的 deg/sec 速率
#define MAX_RATE 1800
#define MAX_RATEYAW 1800

#define LOW_RATES_MULTI 0.5

#define POLAR_EXPO 0.3 // 0.0 .. linear, 1.0 .. square curve

// 自稳模式的最大角度
#define LEVEL_MAX_ANGLE 80

// 级别 pid 使用的最大速率（限制）
#define LEVEL_MAX_RATE 1800

// 更改此系数以获得正确的电池电压。
#define ADC_SCALEFACTOR 11.111 // 11.0 for an ideal 10k/1k voltage divider

// 允许自动检测电池计数（最长 6S）并报告单个电池电压。
#define BATTERY_CELL_COUNT_DETECTION

// 如果电池电量过低，请勿启动软件。 启动时反复闪烁 2 次。
//#define STOP_LOWBATTERY // If below 3.3 Volt

// 如果启用，则在电池电压低时启动 LED 闪烁
#define WARN_ON_LOW_BATTERY 3.6 // Volt
// WARN_ON_LOW_BATTERY电压迟滞
#define VOLTAGE_HYSTERESIS 0.10 // Volt

// 电池电压与节气门下降的补偿
#define VDROP_FACTOR 0.7
// 自动计算上述因素
#define AUTO_VDROP_FACTOR

// 当电池低于阈值时，油门较低
#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// 在没有连接电池的情况下通过 USB 供电时进入 DFU 模式
#define AUTO_BOOTLOADER

// MPU-60x0 片上陀螺仪 LPF 滤波器频率
// gyro filter 0: 256 Hz, delay 0.98 ms (use this to get 8k gyro update frequency)
// gyro filter 1: 188 Hz, delay 1.9 ms
// gyro filter 2: 98 Hz, delay 2.8 ms
// gyro filter 3: 42 Hz, delay 4.8 ms
#define GYRO_LOW_PASS_FILTER 0

// 陀螺仪缺口过滤器
#define RPM_FILTER // requires DSHOT_DMA_BIDIR in hardware.h -- also ensure MOTOR_POLE_COUNT in drv_dshot_bidir.c is correct
#define RPM_FILTER_HZ_MIN 80 // do not apply RPM filtering below RPM_FILTER_HZ_MIN
#define RPM_FILTER_HZ_FADE 40 // gradually increase notch filtering until RPM_FILTER_HZ_MIN + RPM_FILTER_HZ_FADE is reached
#define RPM_FILTER_2ND_HARMONIC true // note, that there are 12 notch filters (4 motors * 3 axes) per harmonic
#define RPM_FILTER_3RD_HARMONIC true
#define RPM_FILTER_Q 6 // -3dB bandwidth = f0 / Q -- but a higher Q also results in a longer settling time

//#define BIQUAD_NOTCH_A_HZ 273 // Dalprop Cyclone T5249C
//#define BIQUAD_NOTCH_A_Q 6

//#define BIQUAD_NOTCH_B_HZ 308 // T-Motor T5147
//#define BIQUAD_NOTCH_B_Q 6

//#define BIQUAD_NOTCH_C_HZ 256 // GemFan WinDacer 51433
//#define BIQUAD_NOTCH_C_Q 6

//#define BIQUAD_AUTO_NOTCH // Performs an FFT of the last second of stored gyro data with the RRR gesture.
//#define BIQUAD_AUTO_NOTCH_Q 6 // (frequency bin resolution is defined via FFT_SIZE in fft.h)

#define BIQUAD_SDFT_NOTCH // Sliding DFT dynamic notch filters. Two per filtered axis.

//#define SDFT_GYRO_FILTER // Experimental frequency domain filtering. Does not work well.

// 动态陀螺仪一阶和二阶有限合力平板

//#define GYRO_LPF_1ST_HZ_BASE 120 // Filter frequency at zero throttle.
//#define GYRO_LPF_1ST_HZ_MAX 120 // A higher filter frequency than loopfrequency/2.4 causes ripples.
//#define GYRO_LPF_1ST_HZ_THROTTLE 0.25 // MAX reached at 1/4 throttle.

//#define GYRO_LPF_2ND_HZ_BASE 240 //* ( aux[ FN_INVERTED ] ? 0.75f : 1.0f )
//#define GYRO_LPF_2ND_HZ_MAX 240
//#define GYRO_LPF_2ND_HZ_THROTTLE 0.25

// 静态陀螺仪一阶LPF
//#define GYRO_LPF_1ST_HZ 240

// 仅在偏航时附加静态陀螺仪一阶 LPF
//#define GYRO_YAW_LPF_1ST_HZ 240

// 卡尔曼陀螺过滤器。指定的值不是 Hz，但会影响过程噪声协方差。
//#define GYRO_KALMAN_q 100 // Higher value is less filtering

// 动态 D 项二阶 LPF
//#define DTERM_LPF_2ND_HZ_BASE 60 //* ( aux[ FN_INVERTED ] ? 0.75f : 1.0f )
//#define DTERM_LPF_2ND_HZ_MAX 60
//#define DTERM_LPF_2ND_HZ_THROTTLE 0.5

// 是否对 D 术语使用贝塞尔类型筛选器，而不是 PT2。
//#define DTERM_BESSEL_FILTER

// 静态 D 项一阶 LPF
#define DTERM_LPF_1ST_A_HZ 60
#define DTERM_LPF_1ST_B_HZ 120

// D-期限峰值
//#define BIQUAD_PEAK_HZ 12
//#define BIQUAD_PEAK_Q 3
//#define BIQUAD_PEAK_GAIN 1.2

// 如果启用，D-Term 滤波器将使用来自上方的 LPF 滤波陀螺仪信号。（如果启用，则始终应用 RPM、陷波和 SDFT 筛选。
//#define CASCADE_GYRO_AND_DTERM_FILTER

// 电机一阶有限合伙油量
//#define MOTOR_FILTER_A_HZ 120
//#define MOTOR_FILTER_B_HZ 240
//#define MOTOR_FILTER_HZ_MULTIPLIER 1 // Multiply motor filter frequency by MOTOR_FILTER_HZ_MULTIPLIER
//#define MOTOR_FILTER_THROTTLE_BREAKPOINT 0.25 // at and above MOTOR_FILTER_THROTTLE_BREAKPOINT.

// 将电机最大速度变化率从零MIX_CHANGE_LIMIT限制在全电机转速下为2 * MIX_CHANGE_LIMIT
//#define MIX_CHANGE_LIMIT 25 // 25/s == 25%/10ms
// 后MIX_CHANGE_LIMIT电机滤波器
//#define MIX_FILTER_HZ 120 // MIX_FILTER_HZ at zero to 2 * MIX_FILTER_HZ at full motor speed
// 卡尔曼电机滤清器。指定的值不是 Hz，但会影响过程噪声协方差。
#define MOTOR_KALMAN_q 100 // Higher value is less filtering

// 开关功能选择

#define RATES DEVO_CHAN_9 // LOW_RATES_MULTI gets applied when RATES is 0.

#define LEVELMODE DEVO_CHAN_10 // Set this to CH_ON for level mode always on. Comment it out for just acro.

#define LEDS_ON DEVO_CHAN_7

// 要停止接地电机，必须设置遥控器上的开关。
#define THROTTLE_KILL_SWITCH DEVO_CHAN_5

// 启用倒置 （3D） 飞行代码
#define INVERTED_ENABLE // goes together with BIDIRECTIONAL in drv_dshot.c / drv_dshot_dma.c / drv_dshot_bidir.c
#define FN_INVERTED DEVO_CHAN_6
//#define LEVEL_MODE_INVERTED_ENABLE // be careful when enabling this

// 通过手势切换两个通道：CH_AUX1和CH_AUX2
// Channel CH_AUX1 changed via gestures LLU -> 1 and LLD -> 0
// Channel CH_AUX2 changed via gestures RRU -> 1 and RRD -> 0
//#define AUX1_START_ON // CH_AUX1 channel starts on if this is defined, otherwise off.
//#define AUX2_START_ON // CH_AUX2 channel starts on if this is defined, otherwise off.

// 使用电机发出四声蜂鸣音（60 秒超时或通过通道）
#define MOTOR_BEEPS
#define MOTOR_BEEPS_CHANNEL DEVO_CHAN_12

// 在遥测数据中发送最大测量的重力。
#define DISPLAY_MAX_G_INSTEAD_OF_VOLTAGE
// 在遥测数据中发送最长循环时间。
#define DISPLAY_MAX_USED_LOOP_TIME_INSTEAD_OF_RX_PACKETS
// 在遥测数据中发送 PID 值。
#define DISPLAY_PID_VALUES

// 无线电模块和协议选择（仅实现巴杨协议）
#define RX_NRF24_BAYANG_TELEMETRY // For nRF24L01+ radio module
//#define RX_XN297_BAYANG_TELEMETRY // For XN297 radio module harvested from toy TX

#define RADIO_XN297 // also enable RX_HARDSPI or RX_SOFTSPI_4WIRE in hardware.h
//#define RADIO_XN297L // also enable RX_SOFTSPI_3WIRE in hardware.h

#define TX_POWER 3 // 0 .. 3 (use 1 when using an nRF24L01+PA+LNA module)

#define AUX_ANALOG_DESTMAX 255 // only for custom DeviationTX builds

// LED 亮度 0 ..15（仅用于实心灯）
#define LED_BRIGHTNESS 15

// 使油门感觉更快的过滤器（又名油门加速）（在LOW_RATES或低电击中不活跃）
#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 3.0

// 补偿节气门以降低电池电压
#define THROTTLE_VOLTAGE_COMPENSATION

//在 3D 飞行中实现更一致的电机反转
#define THROTTLE_REVERSING_KICK 0.15 // 4S
#define THROTTLE_REVERSING_DEADTIME 20000 // 20 ms (increase this in case of over-propped motors)
//#define THROTTLE_STARTUP_KICK 0.05

// 在丢失数据包的情况下，以当前摇杆速度继续摇杆移动
#define RX_PREDICTOR
// 在 RX 信号的 5 ms 阶梯步长之间添加线性插值
#define RX_SMOOTHING
// 将最大斗杆速度从中心周围STICK_VELOCITY_LIMIT限制在完全偏转时的2 * STICK_VELOCITY_LIMIT
#define STICK_VELOCITY_LIMIT 7 // deflection/s (It takes 1/STICK_VELOCITY_LIMIT seconds to reach full stick deflection)
// 将 LPF 应用于滚动、俯仰和偏航摇杆
//#define STICK_FILTER_HZ 10

//Betaflight like mix scaling（又名 Airmode）
#define MIX_SCALING
// 增加混音可产生更清晰的响应，但在低转速下也能产生更跳跃的四边形
#define ALLOW_MIX_INCREASING
// 值越高意味着主动增加周期越短（弹性周期越短）
#define TRANSIENT_MIX_INCREASING_HZ 2.0
// 可用于限制最大电机转速，即调低过快的四边形。
#define MIX_RANGE_LIMIT 1.0f // aux[ DEVO_CHAN_11 ] ? 0.75f : 1.0f

// 调整MOTOR_IDLE_OFFSET，使电机在任何情况下都能可靠地旋转。
#define MOTOR_IDLE_OFFSET 0.03 // 4S
// 当油门杆为零时，油门值为零时有一些动态余量用于零油门下的pid混合。
#define THROTTLE_ZERO_VALUE 0.00 // 4S

//使用平方根电机曲线抵消推力 ~ RPM^2
//#define THRUST_LINEARIZATION 0.4 // 0.0 .. no compensation, 1.0 .. full square root curve
#define ALTERNATIVE_THRUST_LINEARIZATION 0.6 // different shape from the above curve; do not enable both together

// 死区可用于消除棒中心抖动和不返回到正好为 0。
#define STICKS_DEADBAND 0.02f

// 油门直接连接到电机进行推力测量
//#define MOTORS_TO_THROTTLE

// 油门直接对电机进行推力测量，作为飞行模式
#define MOTORS_TO_THROTTLE_MODE CH_AUX1

// 通过手势和/或摇杆位置进行PID调谐
//#define PID_GESTURE_TUNING
#define PID_STICK_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

//将陀螺仪校准（与加速度计校准一起）保存到闪光灯（手势DDD）
#define PERSISTENT_GYRO_CAL

// 针对 4S 调整的未缩放 PID 值。注意：与传统的 SilverWare PID_KP值必须乘以 10。
//             { roll  pitch yaw }
#define PID_KP { 0.40, 0.40, 0.4 }
#define PID_KI { 0.40, 0.40, 2.0 }
#define PID_KD { 0.20, 0.20, 0.0 }

// 补偿PID值以降低电池电压
#define PID_VOLTAGE_COMPENSATION

// 反转偏航皮。向外旋转道具时是必需的。
#define INVERT_YAW_PID

//旋转 I 项矢量以获得稳定的偏航轴（也称为 iTerm 旋转）
#define PID_ROTATE_ERRORS

// 翻转后移除滚动和俯仰反弹（又名iTerm Relax）
#define TRANSIENT_WINDUP_PROTECTION
// 当快速停止滚动/俯仰/偏航运动时，消除反弹（但它主要用于偏航）
#define DYNAMIC_ITERM_RESET

// 零油门时完全平滑，随着油门的增加逐渐减少平滑，在RFS_THROTTLE_BREAKPOINT及以上没有平滑。
#define ROLL_FLIP_SMOOTHER // Scale P, I, and D on roll and pitch axes according to gyro speed.
#define RFS_RATE_MIN 180 // °/s, No scaling below RFS_RATE_MIN. Start scaling at RFS_RATE_MIN.
#define RFS_RATE_MAX 720 // °/s, Linear transition to full scaling at and above RFS_RATE_MAX.
#define RFS_P_SCALER 0.5 // Scale P by this factor at and above RFS_RATE_MAX.
#define RFS_I_SCALER 0.0 // Scale I by this factor at and above RFS_RATE_MAX.
#define RFS_D_SCALER 0.5 // Scale D by this factor at and above RFS_RATE_MAX.
#define RFS_THROTTLE_BREAKPOINT 0.5 // No smoothing at and above RFS_THROTTLE_BREAKPOINT.

// 在滚动轴和俯仰轴上以上升的油门线性衰减 P 和 D，直到达到TPDA_VALUEat TPDA_BREAKPOINT.
//#define THROTTLE_PD_ATTENUATION // No scaling at zero throttle. No scaling for inverted flying.
#define TPDA_VALUE 0.8 // Scale P and D by this factor at TPDA_BREAKPOINT.
#define TPDA_BREAKPOINT 0.5 // Constant scaling with TPDA_VALUE at and above this throttle value.

// 在滚动轴和俯仰轴上缩放 P 和 D，以防陀螺仪显示活动（螺旋桨清洗）而不显示设定点（摇杆）移动。
//#define PROP_WASH_REDUCER
#define PROP_WASH_P_SCALER 1.0
#define PROP_WASH_D_SCALER 1.5

// 快速辊子/俯仰杆直接更换电机，以提供更快速的响应
// 0.0f (or commented out) equates D-term on measurement, 1.0f equates D-term on error.
//#define FEED_FORWARD_STRENGTH 1.0f
//#define SMART_FF
//偏航前馈。这是一个绝对值，与 0 无关。从上面看 1 个。
//#define FEED_FORWARD_YAW 0.2f

//循环时间在我们
#define LOOPTIME 250 // 125 us (8k) needs DSHOT 600 in drv_dshot_bidir.c and an STM32F405 board.
// 陶瓷谐振器频率变化的校正
#define WALLTIME_CORRECTION_FACTOR 1.000 // set to >1 to compensate for a too slow resonator

// 我们体内的故障安全时间。在故障保护没有RX信号时将棒状输入设置为零。保持四边形稳定。
#define FAILSAFETIME 150000 // 0.15 seconds
//电机故障安全时间在我们身上。在额外MOTORS_FAILSAFETIME后关闭电机。
#define MOTORS_FAILSAFETIME 3000000 // 3 seconds

// 陀螺仪方向：
// 预期的方向是芯片上左前角的点。
// 指定必须旋转电路板的量，以使点位于左前方。
// 旋转按顺序执行并累积。
// 请注意，电机不会旋转，因此必须参考新的陀螺仪位置。
//#define SENSOR_ROTATE_45_CCW
//#define SENSOR_ROTATE_45_CW
//#define SENSOR_ROTATE_90_CW
//#define SENSOR_ROTATE_90_CCW
//#define SENSOR_ROTATE_180
//#define SENSOR_INVERT // 如果陀螺仪倒置安装，则为必要。对于倒置陀螺仪，
// 预期的方向是芯片上右前角的点。

#if defined FC_BOARD_OMNIBUS || defined FC_BOARD_F4XSD
	#define SENSOR_ROTATE_90_CCW
#elif defined FC_BOARD_NOXE
	#define SENSOR_ROTATE_180
#elif defined FC_BOARD_NOXE_V1
	// no rotation needed
#else
	#error "FC_BOARD_xxx must be defined by the toolchain, e.g. in the Keil project file."
#endif

// 电机位置
#define MOTOR_BL 2
#define MOTOR_FL 1
#define MOTOR_BR 4
#define MOTOR_FR 3

// 对于双向电机方向，电机可以在下面反转。
#define REVERSE_MOTOR_BL false
#define REVERSE_MOTOR_FL false
#define REVERSE_MOTOR_BR false
#define REVERSE_MOTOR_FR false

// 禁用导致 4 次 LED 闪烁的已知陀螺仪的检查。
#define GYRO_CHECK

// 禁用没有 RX 模块的开发检查（3 次 LED 闪光灯）。
#define RADIO_CHECK

// 将各种信息记录到外部连接的 OpenLager 记录器。
//#define BLACKBOX_LOGGING

// OSD 的使用需要使用 Betaflight Configurator 将自定义 SilF4ware.mcm 字体从"实用工具"文件夹上传到 FC。
// 在 OSD 中显示电池状态、飞行时间和 RSSI 信息。
//#define OSD_ENABLE NTSC // PAL or NTSC
//OSD的地平线。
#define ARTIFICIAL_HORIZON // Comment out to disable the artificial horizon completely.
#define CAMERA_FOV 130 // Horizontal camera field of view in ° (needed for correct artificial horizon movement).
