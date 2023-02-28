/**
 ****************************************************************************************************
 * @file        atk_ms53l1m.h
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

#ifndef __ATK_MS53L1M_H
#define __ATK_MS53L1M_H

#include "main.h"
#include "atk_ms53l1m_uart.h"

/* ATK-MS53L1Mģ�鹦���� */
enum
{
    ATK_MS53L1M_FUNCODE_SYS             = 0x00, /* ϵͳ���� */
    ATK_MS53L1M_FUNCODE_BACKRATE        = 0x01, /* �ش��������� */
    ATK_MS53L1M_FUNCODE_BAUDRATE        = 0x02, /* ���������� */
    ATK_MS53L1M_FUNCODE_IDSET           = 0x03, /* �豸��ַ���� */
    ATK_MS53L1M_FUNCODE_MEAUDATA        = 0x05, /* �������ݻ�ȡ */
    ATK_MS53L1M_FUNCODE_OUTPUTSTATUS    = 0x07, /* ����״̬ */
    ATK_MS53L1M_FUNCODE_MEAUMODE        = 0x08, /* ����ģʽ���� */
    ATK_MS53L1M_FUNCODE_CALIMODE        = 0x09, /* У׼ģʽ */
    ATK_MS53L1M_FUNCODE_WORKMODE        = 0x0A, /* ����ģʽ */
    ATK_MS53L1M_FUNCODE_TIMEBUDGET      = 0x0B, /* ��ʱԤ�� */
    ATK_MS53L1M_FUNCODE_TIMRPERIOD      = 0x0D, /* ������� */
    ATK_MS53L1M_FUNCODE_ERRORFRAM       = 0x0F, /* ����֡��Ϣ */
    ATK_MS53L1M_FUNCODE_VERSION         = 0x10, /* �汾��Ϣ */
};

/* ϵͳ���ò��� */
enum
{
    ATK_MS53L1M_SYS_PARAM_RESET         = 0x01, /* ϵͳ�ָ����� */
    ATK_MS53L1M_SYS_RESET               = 0x02, /* ϵͳ��λ */
};

/* �ش��������ò��� */
enum
{
    ATK_MS53L1M_BACKRATE_01HZ           = 0x00, /* 0.1Hz */
    ATK_MS53L1M_BACKRATE_02HZ           = 0x01, /* 0.2Hz */
    ATK_MS53L1M_BACKRATE_05HZ           = 0x02, /* 0.5Hz */
    ATK_MS53L1M_BACKRATE_1HZ            = 0x03, /* 1Hz */
    ATK_MS53L1M_BACKRATE_2HZ            = 0x04, /* 2Hz */
    ATK_MS53L1M_BACKRATE_5HZ            = 0x05, /* 5Hz */
    ATK_MS53L1M_BACKRATE_10HZ           = 0x06, /* 10Hz */
    ATK_MS53L1M_BACKRATE_20HZ           = 0x07, /* 20Hz */
    ATK_MS53L1M_BACKRATE_50HZ           = 0x08, /* 50Hz */
    ATK_MS53L1M_BACKRATE_100HZ          = 0x09, /* 100Hz */
};

enum
{
    ATK_MS53L1M_CALI_MODE_START         = 0x04, /* ��ʼУ׼ */
};

/* ���ڲ��������ò��� */
enum
{
    ATK_MS53L1M_BAUDRATE_2400           = 0x00, /* 2400bps */
    ATK_MS53L1M_BAUDRATE_4800           = 0x01, /* 4800bps */
    ATK_MS53L1M_BAUDRATE_9600           = 0x02, /* 9600bps */
    ATK_MS53L1M_BAUDRATE_19200          = 0x03, /* 19200bps */
    ATK_MS53L1M_BAUDRATE_38400          = 0x04, /* 38400bps */
    ATK_MS53L1M_BAUDRATE_57600          = 0x05, /* 57600bps */
    ATK_MS53L1M_BAUDRATE_115200         = 0x06, /* 115200bps */
    ATK_MS53L1M_BAUDRATE_230400         = 0x07, /* 230400bps */
    ATK_MS53L1M_BAUDRATE_460800         = 0x08, /* 460800bps */
    ATK_MS53L1M_BAUDRATE_921600         = 0x09, /* 921600bps */
};

/* ����ģʽ���ò��� */
enum
{
    ATK_MS53L1M_MEAUMODE_SHORT          = 0x00, /* �̾��� */
    ATK_MS53L1M_MEAUMODE_MIDDLE         = 0x01, /* �о��� */
    ATK_MS53L1M_MEAUMODE_LONG           = 0x02, /* ������ */
};

/* ����ģʽ���ò��� */
enum
{
    ATK_MS53L1M_WORKMODE_NORMAL         = 0x00, /* Normalģʽ */
    ATK_MS53L1M_WORKMODE_MODBUS         = 0x01, /* Modbusģʽ */
    ATK_MS53L1M_WORKMODE_IIC            = 0x02, /* IICģʽ */
};

/* ����֡������ò��� */
enum
{
    ATK_MS53L1M_ERRORFRAM_OFF           = 0x00, /* �ر� */
    ATK_MS53L1M_ERRORFRAM_ON            = 0x01, /* ���� */
};

/* ������� */
#define ATK_MS53L1M_EOK         0   /* û�д��� */
#define ATK_MS53L1M_ERROR       1   /* ���� */
#define ATK_MS53L1M_ETIMEOUT    2   /* ��ʱ���� */
#define ATK_MS53L1M_EFRAME      3   /* ֡���� */
#define ATK_MS53L1M_ECRC        4   /* CRCУ����� */
#define ATK_MS53L1M_EOPT        5   /* �������� */

/* �������� */
uint8_t atk_ms53l1m_read_data(uint16_t addr, uint8_t code, uint8_t len, uint16_t *dat); /* ����ģ�鹦�����ȡ���� */
uint8_t atk_ms53l1m_write_data(uint16_t addr, uint8_t reg, uint8_t dat);                /* ����ģ�鹦����д��1�ֽ����� */
uint8_t atk_ms53l1m_init(uint32_t baudrate, uint16_t *id);                              /* ATK-MS53L1M��ʼ�� */
uint8_t atk_ms53l1m_normal_get_data(uint16_t *dat);                                     /* ATK-MS53L1M Normal����ģʽ��ȡ����ֵ */
uint8_t atk_ms53l1m_modbus_get_data(uint16_t id, uint16_t *dat);                        /* ATK-MS53L1M Modbus����ģʽ��ȡ����ֵ */

#endif
