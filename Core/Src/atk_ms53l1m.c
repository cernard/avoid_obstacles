/**
 ****************************************************************************************************
 * @file        atk_ms53l1m.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L1Mģ����������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� MiniSTM32 V4������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_ms53l1m.h"
#include "usart.h"
#include <string.h>

#define ATK_MS53L1M_MASTER_FRAME_HEAD   0x51    /* ��������֡ͷ */
#define ATK_MS53L1M_SLAVE_FRAME_HEAD    0x55    /* �ӻ�Ӧ��֡ͷ */
#define ATK_MS53L1M_SENSOR_TYPE         0x0B    /* ATK-MS53L1M���������� */

#define ATK_MS53L1M_FRAME_LEN_MAX       270     /* ����֡��󳤶� */
#define ATK_MS53L1M_FRAME_LEN_MIN       8       /* ����֡��С���� */

#define ATK_MS53L1M_OPT_READ            0x00    /* ������ */
#define ATK_MS53L1M_OPT_WRITE           0x01    /* д���� */

/**
 * @brief       ����CRCУ���
 * @param       buf: ���ݻ���
 *              len: ���ݳ���
 * @retval      CRCУ���ֵ
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
 * @brief       �������յ������ݰ�
 * @param       dat: ������ʱ����ȡ��������
 * @retval      ATK_MS53L1M_EOK     : û�д���
 *              ATK_MS53L1M_ETIMEOUT: �������ݳ�ʱ
 *              ATK_MS53L1M_EFRAME  : ֡����
 *              ATK_MS53L1M_ECRC    : CRCУ�����
 *              ATK_MS53L1M_EOPT    : ��������
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
        /* �ȴ�ATK-MS53L1M UART���յ�һ֡���� */
        frame_buf = atk_ms53l1m_uart_rx_get_frame();
        HAL_Delay(1);
        timeout++;
        if (timeout == 1000)
        {
            /* ���ճ�ʱ���� */
            return ATK_MS53L1M_ETIMEOUT;
        }
    }
    
    /* ��ȡ�������ݵĳ��� */
    recv_len = atk_ms53l1m_uart_rx_get_frame_len();
    if ((recv_len < ATK_MS53L1M_FRAME_LEN_MIN) || (recv_len > ATK_MS53L1M_FRAME_LEN_MAX))
    {
        /* ����֡�����쳣���� */
        return ATK_MS53L1M_EFRAME;
    }
    
    /* ����֡ͷ */
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
            /* ֡�쳣 */
            return ATK_MS53L1M_EFRAME;
        }
    } while (1);
    
    frame_head_index = frame_loop;              /* ��¼֡ͷλ�� */
    frame_len = recv_len - frame_head_index;    /* ����֡���� */
    
    if ((frame_len < ATK_MS53L1M_FRAME_LEN_MIN) || (frame_len > ATK_MS53L1M_FRAME_LEN_MAX))
    {
        /* ����֡�����쳣���� */
        return ATK_MS53L1M_EFRAME;
    }
    
    opt_type = frame_buf[frame_head_index + 4]; /* ��ȡ�������� */
    
    
    if (opt_type == 0x00)
    {
        dat_len = frame_buf[frame_head_index + 7];  /* ��ȡ���ݳ��� */
        if ((dat_len + 10) > frame_len)
        {
            /* ֡�쳣 */
            return ATK_MS53L1M_EFRAME;
        }
        
        frame_check_sum = atk_ms53l1m_crc_check_sum(&frame_buf[frame_head_index], dat_len + 8);   /* ����CRCУ��� */
        check_sum = ((uint16_t)frame_buf[frame_head_index + dat_len + 8] << 8) + frame_buf[frame_head_index + dat_len + 9]; /* ��ȡ֡��CRCУ��� */
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
                /* ֡���� */
                return ATK_MS53L1M_EFRAME;
            }
            
            return ATK_MS53L1M_EOK;
        }
        else
        {
            /* CRC���� */
            return ATK_MS53L1M_ECRC;
        }
    }
    else if (opt_type == 0x01)
    {
        frame_check_sum = atk_ms53l1m_crc_check_sum(&frame_buf[frame_head_index], 6);    /* ����CRCУ��� */
        check_sum = ((uint16_t)frame_buf[frame_head_index + 6] << 8) + frame_buf[frame_head_index + 7];
        if (frame_check_sum == check_sum)
        {
            return ATK_MS53L1M_EOK;
        }
        else
        {
            /* CRC���� */
            return ATK_MS53L1M_ECRC;
        }
    }
    else if (opt_type == 0xFF)
    {
        if ((frame_buf[frame_head_index + 2] == 0xFF) && (frame_buf[frame_head_index + 3] == 0xFF))
        {
            frame_check_sum = atk_ms53l1m_crc_check_sum(&frame_buf[frame_head_index], 6);   /* ����CRCУ��� */
            check_sum = ((uint16_t)frame_buf[frame_head_index + 6] << 8) + frame_buf[frame_head_index + 7];
            if (frame_check_sum == check_sum)
            {
                /* �쳣���� */
                return ATK_MS53L1M_EOPT;
            }
            else
            {
                /* CRC���� */
                return ATK_MS53L1M_ECRC;
            }
        }
        else
        {
            /* ֡�쳣 */
            return ATK_MS53L1M_EFRAME;
        }
    }
    else
    {
        /* ֡�쳣 */
        return ATK_MS53L1M_EFRAME;
    }
}

/**
 * @brief       ����ģ�鹦�����ȡ����
 * @param       addr: �豸��ַ
 *              fun_code : ������
 *              len : ���ݳ��ȣ�ȡֵ��Χ��1��2
 *              dat : ��ȡ��������
 * @retval      ATK_MS53L1M_EOK     : û�д���
 *              ATK_MS53L1M_ETIMEOUT: �������ݳ�ʱ
 *              ATK_MS53L1M_EFRAME  : ֡����
 *              ATK_MS53L1M_ECRC    : CRCУ�����
 *              ATK_MS53L1M_EOPT    : ��������
 */
uint8_t atk_ms53l1m_read_data(uint16_t addr, uint8_t fun_code, uint8_t len, uint16_t *dat)
{
    uint8_t ret;
    uint16_t check_sum;
    uint8_t buf[9];
    
    buf[0] = ATK_MS53L1M_MASTER_FRAME_HEAD;         /* ��־ͷ */
    buf[1] = ATK_MS53L1M_SENSOR_TYPE;               /* ���������� */
    buf[2] = (uint8_t)(addr >> 8);                  /* ��������ַ����8λ */
    buf[3] = (uint8_t)(addr & 0xFF);                /* ��������ַ����8λ */
    buf[4] = ATK_MS53L1M_OPT_READ;                  /* ������ */
    buf[5] = fun_code;                              /* ������ */
    buf[6] = len;                                   /* ���ݳ��� */
    
    check_sum = atk_ms53l1m_crc_check_sum(buf, 7);  /* ����CRCУ��� */
    
    buf[7] = (uint8_t)(check_sum >> 8);             /* CRCУ���룬��8λ */
    buf[8] = (uint8_t)(check_sum & 0xFF);           /* CRCУ���룬��8λ */
    
    atk_ms53l1m_uart_rx_restart();                  /* ׼�����¿�ʼ�����µ�һ֡���� */
    atk_ms53l1m_uart_send(buf, 9);                  /* �������� */
    ret = atk_ms53l1m_unpack_recv_data(dat);        /* ����Ӧ������ */
    
    return ret;
}

/**
 * @brief       ����ģ�鹦����д��1�ֽ�����
 * @param       addr     : �豸��ַ
 *              fun_code : ������
 *              dat      : ��д���1�ֽ�����
 * @retval      ATK_MS53L1M_EOK     : û�д���
 *              ATK_MS53L1M_ETIMEOUT: �������ݳ�ʱ
 *              ATK_MS53L1M_EFRAME  : ֡����
 *              ATK_MS53L1M_ECRC    : CRCУ�����
 *              ATK_MS53L1M_EOPT    : ��������
 */
uint8_t atk_ms53l1m_write_data(uint16_t addr, uint8_t fun_code, uint8_t dat)
{
    uint8_t ret;
    uint8_t buf[10];
    uint16_t check_sum;
    
    buf[0] = ATK_MS53L1M_MASTER_FRAME_HEAD;         /* ��־ͷ */
    buf[1] = ATK_MS53L1M_SENSOR_TYPE;               /* ���������� */
    buf[2] = (uint8_t)(addr >> 8);                  /* ��������ַ����8λ */
    buf[3] = (uint8_t)(addr & 0xFF);                /* ��������ַ����8λ */
    buf[4] = ATK_MS53L1M_OPT_WRITE;                 /* д���� */
    buf[5] = fun_code;                              /* ������ */
    buf[6] = 0x01;                                  /* ���ݳ��� */
    buf[7] = dat;                                   /* ���� */
    
    check_sum = atk_ms53l1m_crc_check_sum(buf, 8);  /* ����CRCУ��� */
    
    buf[8] = (uint8_t)(check_sum >> 8);             /* CRCУ���룬��8λ */
    buf[9] = (uint8_t)(check_sum & 0xFF);           /* CRCУ���룬��8λ */
    
    atk_ms53l1m_uart_rx_restart();                  /* ׼�����¿�ʼ�����µ�һ֡���� */
    atk_ms53l1m_uart_send(buf, 10);                 /* �������� */
    ret = atk_ms53l1m_unpack_recv_data(NULL);       /* ����Ӧ������ */
    
    return ret;
}

/**
 * @brief       ATK-MS53L1M��ʼ��
 * @param       baudrate: ATK-MS53L1M UARTͨѶ������
 *              id      : ATK-MS53L1M���豸ID
 * @retval      ATK_MS53L1M_EOK  : ATK-MS53L1M��ʼ���ɹ�������ִ�гɹ�
 *              ATK_MS53L1M_ERROR: ATK-MS53L1M��ʼ��ʧ�ܣ�����ִ��ʧ��
 */
uint8_t atk_ms53l1m_init(uint32_t baudrate, uint16_t *id)
{
    uint8_t i;
    
    /* ATK-MS53L1M UART��ʼ�� */
    atk_ms53l1m_uart_init(baudrate);
    
    /* ��ȡ�豸��ַ */
    i = 0;
    while (atk_ms53l1m_read_data(0xFFFF, ATK_MS53L1M_FUNCODE_IDSET, 2, id) != ATK_MS53L1M_EOK)
    {
        HAL_Delay(100);
        if (++i == 5)
        {
            return ATK_MS53L1M_ERROR;
        }
    }
    
    /* ����ATK-MS53L1Mģ��Ĺ���ģʽΪModbusģʽ */
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
 * @brief       ATK-MS53L1M Normal����ģʽ��ȡ����ֵ
 * @param       dat: ��ȡ���Ĳ���ֵ
 * @retval      ATK_MS53L1M_EOK  : ��ȡ����ֵ�ɹ�
 *              ATK_MS53L1M_ERROR: UARTδ���յ����ݣ���ȡ����ֵʧ��
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
 * @brief       ATK-MS53L1M Modbus����ģʽ��ȡ����ֵ
 * @param       dat: ��ȡ���Ĳ���ֵ
 * @retval      ATK_MS53L1M_EOK  : ��ȡ����ֵ�ɹ�
 *              ATK_MS53L1M_ERROR: UARTδ���յ����ݣ���ȡ����ֵʧ��
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
