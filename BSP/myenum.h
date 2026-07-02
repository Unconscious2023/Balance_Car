#ifndef __MYENUM_H_
#define __MYENUM_H_

#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00

#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr)  *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr    (GPIOA_BASE + 12)
#define GPIOB_ODR_Addr    (GPIOB_BASE + 12)
#define GPIOC_ODR_Addr    (GPIOC_BASE + 12)
#define GPIOD_ODR_Addr    (GPIOD_BASE + 12)
#define GPIOE_ODR_Addr    (GPIOE_BASE + 12)
#define GPIOF_ODR_Addr    (GPIOF_BASE + 12)
#define GPIOG_ODR_Addr    (GPIOG_BASE + 12)

#define GPIOA_IDR_Addr    (GPIOA_BASE + 8)
#define GPIOB_IDR_Addr    (GPIOB_BASE + 8)
#define GPIOC_IDR_Addr    (GPIOC_BASE + 8)
#define GPIOD_IDR_Addr    (GPIOD_BASE + 8)
#define GPIOE_IDR_Addr    (GPIOE_BASE + 8)
#define GPIOF_IDR_Addr    (GPIOF_BASE + 8)
#define GPIOG_IDR_Addr    (GPIOG_BASE + 8)

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr, n)
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr, n)
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr, n)
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr, n)
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr, n)
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr, n)
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr, n)
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr, n)
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr, n)
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr, n)
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr, n)
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr, n)

typedef enum enCarState_t {
    enSTOP = 0,
    enRUN,
    enBACK,
    enLEFT,
    enRIGHT,
    enTLEFT,
    enTRIGHT,

    // ps2模拟值生效
    enps2Fleft,
    enps2Fright,
    enps2Bleft,
    enps2Bright,

    enAvoid,
    enFollow,
    enError
} enCarState;

typedef enum {
    MOTOR_ID_ML = 0,
    MOTOR_ID_MR,
    MAX_MOTOR
} Motor_ID;

typedef enum Car_mode_t {
    Normal,             // 正常模式
    U_Follow,           // 超声波跟随
    U_Avoid,            // 超声波避障
    Weight_M,           // 负重模式
    PS2_Control,        // PS2控制
    Line_Track,         // 4路红外巡线
    Diff_Line_track,    // 高难度4路巡线
    K210_QR,            // K210识别二维码
    K210_Line,          // K210巡线
    K210_Follow,        // K210跟随
    K210_SelfLearn,     // K210自主学习
    K210_mnist,         // K210识别数字
    LiDar_avoid,        // 雷达避障
    LiDar_Follow,       // 雷达跟随
    LiDar_aralm,        // 雷达警卫
    LiDar_Patrol,       // 雷达巡逻
    LiDar_Line,         // 雷达巡墙边
    LiDar_wall_Line,    // 雷达沿墙走
    CCD_Mode,           // CCD巡线
    ElE_Mode,           // 电磁巡线
    Bluetooth_Mode,     // 蓝牙模式
    ChaseLine_Mode,     // 追逐赛道模式(当前使用)
    Mode_Max
} Car_Mode;

#endif
