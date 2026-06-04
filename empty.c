// /*
//  * ===================================================================
//  * MSPM0G3507 全双工流式高频函数发生器 + 信号采集一体化闭环固件（修复版）
//  * ===================================================================
//  * - 100% 依赖 SysConfig 自动生成的初始化代码
//  * - 接收 (电脑->MCU)：UART0 中断接收 11字节 DAC 帧，写入播放缓冲区
//  * - 发送 (MCU->电脑)：Main 循环异步读取采集缓冲区，打包发送 11字节 ADC 帧
//  * - 定时器 (TIMG0 100kHz)：
//  * 1. 100kHz 准时消费 DAC 缓冲区送 DAC0 (PA15) 播放
//  * 2. 50kHz 降采样触发 ADC12_0 采集并将数据推入采集缓冲区
//  */

// #include "ti_msp_dl_config.h"
// #include <stdint.h>
// #include <stdbool.h>

// /* 【核心修复 1】：显式引入 TI DriverLib 核心外设头文件，消除隐式声明警告 */
// #include <ti/driverlib/dl_timer.h>
// #include <ti/driverlib/dl_dac12.h>
// #include <ti/driverlib/dl_adc12.h>
// #include <ti/driverlib/dl_uart.h>

// /* ========================================================================
//  * 协议与缓冲区常量定义
//  * ======================================================================== */
// #define FRAME_HEAD1          0x5A
// #define FRAME_HEAD2          0xA5
// #define CMD_COMPACT          0x02
// #define SAMPLES_PER_FRAME    3

// /* 环形缓冲区尺寸（2的幂次方优化） */
// #define RING_BUF_SIZE        2048
// #define RING_MASK            (RING_BUF_SIZE - 1)

// /* 1. DAC 接收与播放环形缓冲区 (电脑 -> MCU) */
// static volatile uint16_t g_dac_ring_buf[RING_BUF_SIZE];
// static volatile uint32_t g_dac_head = 0;  /* UART 中断写入 */
// static volatile uint32_t g_dac_tail = 0;  /* Timer 中断读取 */
// static volatile uint32_t g_dac_lost = 0;  /* 缓冲区空/断流计数 */

// /* 2. ADC 采集与上传环形缓冲区 (MCU -> 电脑) */
// static volatile uint16_t g_adc_ring_buf[RING_BUF_SIZE];
// static volatile uint32_t g_adc_head = 0;  /* Timer 中断写入 */
// static volatile uint32_t g_adc_tail = 0;  /* Main 循环读取发送 */
// static volatile uint32_t g_adc_lost = 0;  /* ADC 缓冲区溢出丢点计数 */
// static volatile uint32_t g_adc_sent = 0;  /* 已发送 ADC 帧计数 */
// static volatile uint32_t g_tick_100khz = 0; /* TIMG0 100kHz 节拍 */
// static volatile bool    g_do_status = false; /* 触发状态上报 */

// /* 串口接收状态机 — 全局 volatile（匹配已验证的 ISR 写法） */
// typedef enum {
//     RX_STATE_H1 = 0,
//     RX_STATE_H2,
//     RX_STATE_CH,
//     RX_STATE_CMD,
//     RX_STATE_DATA,
//     RX_STATE_CHKSUM
// } rx_state_t;

// static volatile rx_state_t g_rx_state = RX_STATE_H1;
// static volatile uint8_t    g_rx_buf[11];     /* 11 字节帧缓冲 */
// static volatile uint8_t    g_rx_idx = 0;
// static volatile uint16_t   g_rx_sum = 0;
// static volatile uint32_t   g_frame_ok  = 0;  /* 调试计数 */
// static volatile uint32_t   g_frame_bad = 0;

// /* ========================================================================
//  * 底层驱动辅助函数
//  * ======================================================================== */

// /* ── 前向声明 ── */
// static inline void uart_tx_byte(uint8_t byte);

// /* ── hex 工具 ── */
// static inline char nibble_to_hex(uint8_t n)
// {
//     return (char)((n < 10) ? ('0' + n) : ('A' + (n - 10)));
// }

// static void uart_tx_hex32(uint32_t val)
// {
//     uart_tx_byte((uint8_t)nibble_to_hex((val >> 28) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val >> 24) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val >> 20) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val >> 16) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val >> 12) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val >>  8) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val >>  4) & 0xF));
//     uart_tx_byte((uint8_t)nibble_to_hex((val       & 0xF)));
// }

// /* ── 已验证的串口发送：DL_UART_isBusy + DL_UART_Main_transmitData ── */
// static inline void uart_tx_byte(uint8_t byte)
// {
//     while (DL_UART_isBusy(UART_0_INST)) { }
//     DL_UART_Main_transmitData(UART_0_INST, byte);
// }

// /* 发送 11-byte 紧凑帧 (CMD=0x02)：3 个 12-bit 采样点 + 校验和 */
// void send_adc_frame(uint16_t s0, uint16_t s1, uint16_t s2)
// {
//     uint8_t frame[11];
//     frame[0] = FRAME_HEAD1;
//     frame[1] = FRAME_HEAD2;
//     frame[2] = 0x01;
//     frame[3] = CMD_COMPACT;
//     frame[4] = (uint8_t)(s0 >> 8);  frame[5] = (uint8_t)(s0 & 0xFF);
//     frame[6] = (uint8_t)(s1 >> 8);  frame[7] = (uint8_t)(s1 & 0xFF);
//     frame[8] = (uint8_t)(s2 >> 8);  frame[9] = (uint8_t)(s2 & 0xFF);

//     uint16_t sum = 0;
//     for (int i = 0; i < 10; i++) { sum += frame[i]; }
//     frame[10] = (uint8_t)(sum & 0xFF);

//     for (int i = 0; i < 11; i++) { uart_tx_byte(frame[i]); }
// }

// /* ========================================================================
//  * 主程序入口
//  * ======================================================================== */
// int main(void)
// {
//     /* 1. 初始化 SysConfig 生成的全部外设 */
//     SYSCFG_DL_init();

//     /* 2. 显式开启外设内核级中断 */
//     NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
//     NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    
//     NVIC_ClearPendingIRQ(TIMG_0_INST_INT_IRQN);
//     NVIC_EnableIRQ(TIMG_0_INST_INT_IRQN);
    
//     /* 3. 启动定时器核心计数器 */
//     DL_Timer_startCounter(TIMG_0_INST);
    
//     /* 4. 预触发首次 ADC 转换 */
//     DL_ADC12_startConversion(ADC12_0_INST);

//     /* 5. 开启全局中断响应 */
//     __enable_irq();

//     /* 6. 上电确认：发送 boot 消息，验证 UART TX 通路 */
//     {
//         const char *msg = "MSPM0G3507 Ready\r\n";
//         for (const char *p = msg; *p; p++) {
//             uart_tx_byte((uint8_t)*p);
//         }
//     }

//     while (1)
//     {
//         /* ── 批量回传 ADC 采集帧到 PC 示波器渲染 ── */
//         uint32_t avail = g_adc_head - g_adc_tail;
//         while (avail >= SAMPLES_PER_FRAME)
//         {
//             uint16_t s1 = g_adc_ring_buf[g_adc_tail & RING_MASK]; g_adc_tail++;
//             uint16_t s2 = g_adc_ring_buf[g_adc_tail & RING_MASK]; g_adc_tail++;
//             uint16_t s3 = g_adc_ring_buf[g_adc_tail & RING_MASK]; g_adc_tail++;
//             send_adc_frame(s1, s2, s3);
//             g_adc_sent++;
//             avail -= SAMPLES_PER_FRAME;
//         }

//         /* ── 每秒一次状态上报（由 TIMG0 ISR 的 g_do_status 触发） ── */
//         if (g_do_status) {
//             g_do_status = false;
//             // 格式: "A:<sent> L:<lost> F:<ok> B:<bad>\r\n"
//             uart_tx_byte('A'); uart_tx_byte(':');
//             uart_tx_hex32(g_adc_sent);
//             uart_tx_byte(' '); uart_tx_byte('L'); uart_tx_byte(':');
//             uart_tx_hex32(g_adc_lost);
//             uart_tx_byte(' '); uart_tx_byte('F'); uart_tx_byte(':');
//             uart_tx_hex32(g_frame_ok);
//             uart_tx_byte(' '); uart_tx_byte('B'); uart_tx_byte(':');
//             uart_tx_hex32(g_frame_bad);
//             uart_tx_byte('\r'); uart_tx_byte('\n');
//         }
//     }
// }

// /* ========================================================================
//  * 定时器中断服务函数 (TIMG0 - 严格 100kHz 执行)
//  * ======================================================================== */
// void TIMG_0_INST_IRQHandler(void)
// {
//     static uint8_t adc_divider = 0;

//     switch (DL_Timer_getPendingInterrupt(TIMG_0_INST))
//     {
//         case DL_TIMER_IIDX_ZERO:
//         {
//             /* 100kHz 时钟节拍：每 100,000 次 = 1 秒触发状态上报 */
//             g_tick_100khz++;
//             if (g_tick_100khz >= 100000) {
//                 g_tick_100khz = 0;
//                 g_do_status = true;
//             }

//             /* ---------------- 核心任务 1：100kHz DAC 播放 ---------------- */
//             if (g_dac_head != g_dac_tail) 
//             {
//                 uint16_t dac_data = g_dac_ring_buf[g_dac_tail & RING_MASK];
//                 /* 【核心修复 2】：将原未定义标识符 DAC12_INST 修正为 SysConfig 原生宏名 DAC0 */
//                 DL_DAC12_output12(DAC0, dac_data); 
//                 g_dac_tail++;
//             } 
//             else 
//             {
//                 g_dac_lost++; 
//             }

//             /* ---------------- 核心任务 2：50kHz ADC 采集 (2分频) ---------------- */
//             adc_divider++;
//             if (adc_divider >= 2)
//             {
//                 adc_divider = 0;

//                 uint16_t adc_raw = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);

//                 /* 环形缓冲溢出保护 */
//                 if ((g_adc_head - g_adc_tail) < RING_BUF_SIZE) {
//                     g_adc_ring_buf[g_adc_head & RING_MASK] = adc_raw;
//                     g_adc_head++;
//                 } else {
//                     g_adc_lost++;
//                 }

//                 DL_ADC12_startConversion(ADC12_0_INST);
//             }
//             break;
//         }
//         default:
//             break;
//     }
// }

// /* ========================================================================
//  * 串口接收 ISR — 已验证的 switch/case IIDX + 全局 volatile 状态机写法
//  * ======================================================================== */
// void UART_0_INST_IRQHandler(void)
// {
//     switch (DL_UART_getPendingInterrupt(UART_0_INST))
//     {
//         case DL_UART_IIDX_RX:
//         {
//             uint8_t byte = (uint8_t)DL_UART_Main_receiveData(UART_0_INST);

//             /* ── 逐字节帧头锁定 + 滚动校验和 + 11 字节紧凑帧解析 ── */
//             if (g_rx_idx == 0) {
//                 if (byte == FRAME_HEAD1) {
//                     g_rx_buf[g_rx_idx++] = byte;
//                     g_rx_sum = byte;
//                 }
//             } else if (g_rx_idx == 1) {
//                 if (byte == FRAME_HEAD2) {
//                     g_rx_buf[g_rx_idx++] = byte;
//                     g_rx_sum += byte;
//                 } else {
//                     g_rx_idx = 0;  /* 帧头2不匹配 → 复位 */
//                 }
//             } else if (g_rx_idx == 2) {
//                 if (byte == 0x01) {         /* CH=1 过滤 */
//                     g_rx_buf[g_rx_idx++] = byte;
//                     g_rx_sum += byte;
//                 } else {
//                     g_rx_idx = 0;
//                 }
//             } else if (g_rx_idx == 3) {
//                 if (byte == CMD_COMPACT) {  /* 仅接受 CMD=0x02 */
//                     g_rx_buf[g_rx_idx++] = byte;
//                     g_rx_sum += byte;
//                 } else {
//                     g_rx_idx = 0;
//                 }
//             } else if (g_rx_idx < 10) {
//                 /* 数据字节 [4]..[9]：3 个 uint16 大端 DAC 值 */
//                 g_rx_buf[g_rx_idx++] = byte;
//                 g_rx_sum += byte;
//             } else {
//                 /* g_rx_idx == 10：校验和字节 */
//                 g_rx_buf[g_rx_idx] = byte;

//                 if ((uint8_t)(g_rx_sum & 0xFF) == byte) {
//                     uint16_t s1 = (g_rx_buf[4] << 8) | g_rx_buf[5];
//                     uint16_t s2 = (g_rx_buf[6] << 8) | g_rx_buf[7];
//                     uint16_t s3 = (g_rx_buf[8] << 8) | g_rx_buf[9];

//                     if ((g_dac_head - g_dac_tail) < (RING_BUF_SIZE - 4)) {
//                         g_dac_ring_buf[g_dac_head & RING_MASK] = s1; g_dac_head++;
//                         g_dac_ring_buf[g_dac_head & RING_MASK] = s2; g_dac_head++;
//                         g_dac_ring_buf[g_dac_head & RING_MASK] = s3; g_dac_head++;
//                     }
//                     g_frame_ok++;
//                 } else {
//                     g_frame_bad++;
//                 }
//                 g_rx_idx = 0;  /* 无论成败，复位准备下一帧 */
//             }
//             break;
//         }

//         default:
//             break;
//     }
// }


/*
 * ===================================================================
 * MSPM0G3507 11字节波形数据包【状态机解析与校验】压测固件
 * ===================================================================
 * - 严格按照你的代码规范与 API 编写
 * - 采用 11 字节紧凑帧协议 (HEAD1=0x5A, HEAD2=0xA5)
 * - 硬件中断内集成高可靠性状态机，拒绝一切噪点和错位数据
 * - 校验通过（Checksum正确）的数据才会触发回传，用于终极链路测试
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* 协议常量定义 */
#define FRAME_HEAD1     0x5A
#define FRAME_HEAD2     0xA5
#define FRAME_LEN       11

/* 状态机及接收缓冲区变量 */
volatile uint8_t  g_rx_buffer[FRAME_LEN]; // 接收帧缓冲区
volatile uint8_t  g_rx_index = 0;         // 状态机步骤/索引指针
volatile uint32_t g_valid_frame_cnt = 0;  // 成功校验通过的帧计数器（调试用）

/* 函数声明 (完全对齐你的代码规范) */
void uart0_send_char(char ch);     // 串口0发送单个字符
void uart0_send_string(char* str); // 串口0发送字符串

int main(void)
{
    /* 初始化 SysConfig 生成的所有基本外设 */
    SYSCFG_DL_init();

    /* 1. 清除串口中断标志 */
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    
    /* 2. 使能串口中断 */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    
    /* 3. 开启 CPU 全局中断响应 */
    __enable_irq();

    /* 提示上位机：单片机校验固件已就绪 */
    uart0_send_string("Packet Validation Firmware Ready...\r\n");

    while (1)
    {
        /* 主循环保持空闲，所有高性能的流式解析和校验回传都在中断状态机内完成 */
    }
}

/* 串口发送单个字符 (完全采用你的忙等待与硬件发送逻辑) */
void uart0_send_char(char ch)
{
    // 当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );
    
    // 发送单个字符
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

/* 串口发送字符串 (完全采用你的逻辑) */
void uart0_send_string(char* str)
{
    while(*str != 0 && str != 0)
    {
        uart0_send_char(*str++);
    }
}

/* 串口的中断服务函数 (集成 11 字节流式校验状态机) */
void UART_0_INST_IRQHandler(void)
{
    // 获取当前待处理的串口中断类型
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX: // 如果是接收中断
        {
            // 1. 读取接收寄存器中的当前单字节
            uint8_t data = (uint8_t)DL_UART_Main_receiveData(UART_0_INST);
            
            // 2. 流式数据包解析状态机
            if (g_rx_index == 0) 
            {
                // 寻找第一个帧头 0x5A
                if (data == FRAME_HEAD1) {
                    g_rx_buffer[g_rx_index++] = data;
                }
            } 
            else if (g_rx_index == 1) 
            {
                // 寻找第二个帧头 0xA5
                if (data == FRAME_HEAD2) {
                    g_rx_buffer[g_rx_index++] = data;
                } else {
                    g_rx_index = 0; // 帧头2不匹配，判定为噪点，重态复位
                }
            } 
            else if (g_rx_index < (FRAME_LEN - 1)) 
            {
                // 填充中间的数据位 (通道号、命令字、3个采样点共6字节)
                g_rx_buffer[g_rx_index++] = data;
            } 
            else if (g_rx_index == (FRAME_LEN - 1)) 
            {
                // 此时收到了第 11 个字节（第10项索引），即 Checksum 校验位
                g_rx_buffer[g_rx_index] = data;
                
                // 计算前 10 个字节的严格累加和
                uint16_t sum = 0;
                for (int i = 0; i < 10; i++) {
                    sum += g_rx_buffer[i];
                }
                
                // 核心检验：比对计算出的低8位与收到的校验字节是否绝对一致
                if ((uint8_t)(sum & 0xFF) == data) 
                {
                    g_valid_frame_cnt++; // 校验成功，计数器自增
                    
                    // 【打靶通过】：将这帧经过严格认证的 11 字节完整回发给电脑
                    for (int i = 0; i < FRAME_LEN; i++) {
                        uart0_send_char((char)g_rx_buffer[i]);
                    }
                }
                
                // 无论校验成功还是失败，均清空索引，准备解析下一个数据包
                g_rx_index = 0;
            }
            break;
        }
            
        default:
            break;
    }
}
