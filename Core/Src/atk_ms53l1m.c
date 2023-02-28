/**
 ****************************************************************************************************
 * @file        atk_ms53l1m.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L1M模块驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 MiniSTM32 V4开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_ms53l1m.h"
#include "usart.h"
#include <string.h>

#define ATK_MS53L1M_MASTER_FRAME_HEAD   0x51    /* 主机请求帧头 */
#define ATK_MS53L1M_SLAVE_FRAME_HEAD    0x55    /* 从机应答帧头 */
#define ATK_MS53L1M_SENSOR_TYPE         0x0B    /* ATK-MS53L1M传感器类型 */

#define ATK_MS53L1M_FRAME_LEN_MAX       270     /* 接收帧最大长度 */
#define ATK_MS53L1M_FRAME_LEN_MIN       8       /* 接收帧最小长度 */

#define ATK_MS53L1M_OPT_READ            0x00    /* 读操作 */
#define ATK_MS53L1M_OPT_WRITE           0x01    /* 写操作 */

/**
 * @brief       计算CRC校验和
 * @param       buf: 数据缓冲
 *              len: 数据长度
 * @retval      CRC校验和值
 */
static inline uint16_t atk_ms53l1m_crc_check_sum(uint8_t *buf, uint16_t len)
{
    uint16_t check_sum = 0;
    uint16_t i;
    
    for (i=0; i<len; i++)
    {
        check_sum += buf[i];
    }
    
    return check_sum;
}

/**
 * @brief       解析接收到的数据包
 * @param       dat: 读操作时，读取到的数据
 * @retval      ATK_MS53L1M_EOK     : 没有错误
 *              ATK_MS53L1M_ETIMEOUT: 接收数据超时
 *              ATK_MS53L1M_EFRAME  : 帧错误
 *              ATK_MS53L1M_ECRC    : CRC校验错误
 *              ATK_MS53L1M_EOPT    : 操作错误
 */
static uint8_t atk_ms53l1m_unpack_recv_data(uint16_t *dat)
{
    uint8_t *frame_buf = NULL;
    uint16_t timeout = 0;
    uint16_t recv_len = 0;
    uint16_t frame_loop = 0;
    uint16_t frame_head_index;
    uint16_t frame_len;
    uint8_t opt_type;
    uint16_t dat_len;
    uint16_t frame_check_sum;
    uint16_t check_sum;
    
    while (frame_buf == NULL)
    {
        /* 等待ATK-MS53L1M UART接收到一帧数据 */
        frame_buf = atk_ms53l1m_uart_rx_get_frame();
        HAL_Delay(1);
        timeout++;
        if (timeout == 1000)
        {
            /* 接收超时错误 */
            return ATK_MS53L1M_ETIMEOUT;
        }
    }
    
    /* 获取接收数据的长度 */
    recv_len = atk_ms53l1m_uart_rx_get_frame_len();
    if ((recv_len < ATK_MS53L1M_FRAME_LEN_MIN) || (recv_len > ATK_MS53L1M_FRAME_LEN_MAX))
    {
        /* 接收帧长度异常错误 */
        return ATK_MS53L1M_EFRAME;
    }
    
    /* 查找帧头 */
    do
    {
        if ((frame_buf[frame_loop] == ATK_MS53L1M_SLAVE_FRAME_HEAD) && 
            (frame_buf[frame_loop + 1] == ATK_MS53L1M_SENSOR_TYPE))
        {
            break;
        }
        
        if (frame_loop != (recv_len - 2))
        {
            frame_loop++;
        }
        else
        {
            /* 帧异常 */
            return ATK_MS53L1M_EFRAME;
        }
    } while (1);
    
    frame_head_index = frame_loop;              /* 记录帧头位置 */
    frame_len = recv_len - frame_head_index;    /* 计算帧长度 */
    
    if ((frame_len < ATK_MS53L1M_FRAME_LEN_MIN) || (frame_len > ATK_MS53L1M_FRAME_LEN_MAX))
    {
        /* 接收帧长度异常错误 */
        return ATK_MS53L1M_EFRAME;
    }
    
    opt_type = frame_buf[frame_head_index + 4]; /* 获取操作类型 */
    
    
    if (opt_type == 0x00)
    {
        dat_len = frame_buf[frame_head_index + 7];  /* 获取数据长度 */
        if ((dat_len + 10) > frame_len)
        {
            /* 帧异常 */
            return ATK_MS53L1M_EFRAME;
        }
        
        frame_check_sum = atk_ms53l1m_crc_check_sum(&frame_buf[frame_head_index], dat_len + 8);   /* 计算CRC校验和 */
        check_sum = ((uint16_t)frame_buf[frame_head_index + dat_len + 8] << 8) + frame_buf[frame_head_index + dat_len + 9]; /* 获取帧的CRC校验和 */
        if (frame_check_sum == check_sum)
        {
            if (frame_buf[frame_head_index + 7] == 1)
            {
                *dat = frame_buf[frame_head_index + 8];
            }
            else if (frame_buf[frame_head_index + 7] == 2)
            {
                *dat = ((uint16_t)frame_buf[frame_head_index + 8] << 8) + frame_buf[frame_head_index + 9];
            }
            else
            {
                /* 帧错误 */
                return ATK_MS53L1M_EFRAME;
            }
            
            return ATK_MS53L1M_EOK;
        }
        else
        {
            /* CRC错误 */
            return ATK_MS53L1M_ECRC;
        }
    }
    else if (opt_type == 0x01)
    {
        frame_check_sum = atk_ms53l1m_crc_check_sum(&frame_buf[frame_head_index], 6);    /* 计算CRC校验和 */
        check_sum = ((uint16_t)frame_buf[frame_head_index + 6] << 8) + frame_buf[frame_head_index + 7];
        if (frame_check_sum == check_sum)
        {
            return ATK_MS53L1M_EOK;
        }
        else
        {
            /* CRC错误 */
            return ATK_MS53L1M_ECRC;
        }
    }
    else if (opt_type == 0xFF)
    {
        if ((frame_buf[frame_head_index + 2] == 0xFF) && (frame_buf[frame_head_index + 3] == 0xFF))
        {
            frame_check_sum = atk_ms53l1m_crc_check_sum(&frame_buf[frame_head_index], 6);   /* 计算CRC校验和 */
            check_sum = ((uint16_t)frame_buf[frame_head_index + 6] << 8) + frame_buf[frame_head_index + 7];
            if (frame_check_sum == check_sum)
            {
                /* 异常操作 */
                return ATK_MS53L1M_EOPT;
            }
            else
            {
                /* CRC错误 */
                return ATK_MS53L1M_ECRC;
            }
        }
        else
        {
            /* 帧异常 */
            return ATK_MS53L1M_EFRAME;
        }
    }
    else
    {
        /* 帧异常 */
        return ATK_MS53L1M_EFRAME;
    }
}

/**
 * @brief       根据模块功能码读取数据
 * @param       addr: 设备地址
 *              fun_code : 功能码
 *              len : 数据长度，取值范围：1或2
 *              dat : 读取到的数据
 * @retval      ATK_MS53L1M_EOK     : 没有错误
 *              ATK_MS53L1M_ETIMEOUT: 接收数据超时
 *              ATK_MS53L1M_EFRAME  : 帧错误
 *              ATK_MS53L1M_ECRC    : CRC校验错误
 *              ATK_MS53L1M_EOPT    : 操作错误
 */
uint8_t atk_ms53l1m_read_data(uint16_t addr, uint8_t fun_code, uint8_t len, uint16_t *dat)
{
    uint8_t ret;
    uint16_t check_sum;
    uint8_t buf[9];
    
    buf[0] = ATK_MS53L1M_MASTER_FRAME_HEAD;         /* 标志头 */
    buf[1] = ATK_MS53L1M_SENSOR_TYPE;               /* 传感器类型 */
    buf[2] = (uint8_t)(addr >> 8);                  /* 传感器地址，高8位 */
    buf[3] = (uint8_t)(addr & 0xFF);                /* 传感器地址，低8位 */
    buf[4] = ATK_MS53L1M_OPT_READ;                  /* 读操作 */
    buf[5] = fun_code;                              /* 功能码 */
    buf[6] = len;                                   /* 数据长度 */
    
    check_sum = atk_ms53l1m_crc_check_sum(buf, 7);  /* 计算CRC校验和 */
    
    buf[7] = (uint8_t)(check_sum >> 8);             /* CRC校验码，高8位 */
    buf[8] = (uint8_t)(check_sum & 0xFF);           /* CRC校验码，低8位 */
    
    atk_ms53l1m_uart_rx_restart();                  /* 准备重新开始接收新的一帧数据 */
    atk_ms53l1m_uart_send(buf, 9);                  /* 发送数据 */
    ret = atk_ms53l1m_unpack_recv_data(dat);        /* 解析应答数据 */
    
    return ret;
}

/**
 * @brief       根据模块功能码写入1字节数据
 * @param       addr     : 设备地址
 *              fun_code : 功能码
 *              dat      : 待写入的1字节数据
 * @retval      ATK_MS53L1M_EOK     : 没有错误
 *              ATK_MS53L1M_ETIMEOUT: 接收数据超时
 *              ATK_MS53L1M_EFRAME  : 帧错误
 *              ATK_MS53L1M_ECRC    : CRC校验错误
 *              ATK_MS53L1M_EOPT    : 操作错误
 */
uint8_t atk_ms53l1m_write_data(uint16_t addr, uint8_t fun_code, uint8_t dat)
{
    uint8_t ret;
    uint8_t buf[10];
    uint16_t check_sum;
    
    buf[0] = ATK_MS53L1M_MASTER_FRAME_HEAD;         /* 标志头 */
    buf[1] = ATK_MS53L1M_SENSOR_TYPE;               /* 传感器类型 */
    buf[2] = (uint8_t)(addr >> 8);                  /* 传感器地址，高8位 */
    buf[3] = (uint8_t)(addr & 0xFF);                /* 传感器地址，低8位 */
    buf[4] = ATK_MS53L1M_OPT_WRITE;                 /* 写操作 */
    buf[5] = fun_code;                              /* 功能码 */
    buf[6] = 0x01;                                  /* 数据长度 */
    buf[7] = dat;                                   /* 数据 */
    
    check_sum = atk_ms53l1m_crc_check_sum(buf, 8);  /* 计算CRC校验和 */
    
    buf[8] = (uint8_t)(check_sum >> 8);             /* CRC校验码，高8位 */
    buf[9] = (uint8_t)(check_sum & 0xFF);           /* CRC校验码，低8位 */
    
    atk_ms53l1m_uart_rx_restart();                  /* 准备重新开始接收新的一帧数据 */
    atk_ms53l1m_uart_send(buf, 10);                 /* 发送数据 */
    ret = atk_ms53l1m_unpack_recv_data(NULL);       /* 解析应答数据 */
    
    return ret;
}

/**
 * @brief       ATK-MS53L1M初始化
 * @param       baudrate: ATK-MS53L1M UART通讯波特率
 *              id      : ATK-MS53L1M的设备ID
 * @retval      ATK_MS53L1M_EOK  : ATK-MS53L1M初始化成功，函数执行成功
 *              ATK_MS53L1M_ERROR: ATK-MS53L1M初始化失败，函数执行失败
 */
uint8_t atk_ms53l1m_init(uint32_t baudrate, uint16_t *id)
{
    uint8_t i;
    
    /* ATK-MS53L1M UART初始化 */
    atk_ms53l1m_uart_init(baudrate);
    
    /* 获取设备地址 */
    i = 0;
    while (atk_ms53l1m_read_data(0xFFFF, ATK_MS53L1M_FUNCODE_IDSET, 2, id) != ATK_MS53L1M_EOK)
    {
        HAL_Delay(100);
        if (++i == 5)
        {
            return ATK_MS53L1M_ERROR;
        }
    }
    
    /* 设置ATK-MS53L1M模块的工作模式为Modbus模式 */
    i = 0;
    while (atk_ms53l1m_write_data(*id, ATK_MS53L1M_FUNCODE_WORKMODE, ATK_MS53L1M_WORKMODE_MODBUS) != ATK_MS53L1M_EOK)
    {
        HAL_Delay(100);
        if (++i == 5)
        {
            return ATK_MS53L1M_ERROR;
        }
    }
    
    return ATK_MS53L1M_EOK;
}

/**
 * @brief       ATK-MS53L1M Normal工作模式获取测量值
 * @param       dat: 获取到的测量值
 * @retval      ATK_MS53L1M_EOK  : 获取测量值成功
 *              ATK_MS53L1M_ERROR: UART未接收到数据，获取测量值失败
 */
uint8_t atk_ms53l1m_normal_get_data(uint16_t *dat)
{
    uint8_t *buf = NULL;
    uint8_t i = 0;
    char *p;
    uint16_t dat_tmp = 0;
    
    atk_ms53l1m_uart_rx_restart();
    while (buf == NULL)
    {
        buf = atk_ms53l1m_uart_rx_get_frame();
        if (++i == 10)
        {
            return ATK_MS53L1M_ERROR;
        }
        HAL_Delay(100);
    }
    
    p = strstr((char *)buf, "d:");
    while (*p != 'm')
    {
        if (*p >= '0' && *p <= '9')
        {
            dat_tmp = dat_tmp * 10 + (*p - '0');
        }
        p++;
    }
    
    *dat = dat_tmp;
    
    return ATK_MS53L1M_EOK;
}

/**
 * @brief       ATK-MS53L1M Modbus工作模式获取测量值
 * @param       dat: 获取到的测量值
 * @retval      ATK_MS53L1M_EOK  : 获取测量值成功
 *              ATK_MS53L1M_ERROR: UART未接收到数据，获取测量值失败
 */
uint8_t atk_ms53l1m_modbus_get_data(uint16_t id, uint16_t *dat)
{
    uint8_t ret;
    uint16_t dat_tmp;
    
    ret = atk_ms53l1m_read_data(id, ATK_MS53L1M_FUNCODE_MEAUDATA, 2, &dat_tmp);
    if (ret != 0)
    {
        *dat = 0;
        return ATK_MS53L1M_ERROR;
    }
    else
    {
        *dat = dat_tmp;
        return ATK_MS53L1M_EOK;
    }
}
