/**
 * @file param.c
 * @brief ��������ģ��ʵ��
 * 
 * ʹ��STM32�ڲ�Flashģ��EEPROM���������洢��ָ��������
 * д��ʱͬʱд���������ݣ���ȡʱ���������
 */

#include "param.h"
#include "config.h"
#include <string.h>
#include <stddef.h>

#include "stm32h7xx_hal.h"

// Flash�������
#define PARAM_BACKUP_COUNT     3		//3������ 
#define PARAM_SECTOR_SIZE      0x20000     // H7ϵ��һ������Ϊ128KB��0x20000ת��Ϊ10���ƺ��Ӧ�ֽ���Ϊ128KB 

// �ڲ�����������ֻ�ڱ��ļ���ʹ�� 
static uint16_t crc16(const uint8_t* data, uint32_t len);  //CRCУ�麯�� 
/*
CRC��Cyclic Redundancy Check��ѭ������У�飩
ԭ��
��ҪУ������ݣ�����һ���ֽڣ�����һ�������ƶ���ʽ����һ���̶��ġ����ɶ���ʽ����ģ 2 �������õ�һ��������
����������� CRC ֵ�����ͷ����� CRC ��������һ���͡�
���շ��յ����ݺ���ͬ�����㷨����һ�� CRC�����յ��� CRC �Ƚϣ����һ�£�˵�����ݴ����û���۸Ļ��𻵡�
*/
static ParamStatus read_params(SystemParams* params);	//flasg��ȡ 
static ParamStatus write_params(const SystemParams* params) {
    HAL_FLASH_Unlock();

    // 擦除三个扇区（使用扇区编号）
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = PARAM_FLASH_SECTOR;
    erase.NbSectors = PARAM_BACKUP_COUNT;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return PARAM_ERROR_WRITE_FAILED;
    }

    // 编程三个备份
    uint32_t base = PARAM_FLASH_ADDR;
    for (int i = 0; i < PARAM_BACKUP_COUNT; ++i) {
        uint32_t addr = base + i * PARAM_SECTOR_SIZE;
        const uint32_t* src = (const uint32_t*)params;
        for (uint32_t j = 0; j < sizeof(SystemParams) / 4; ++j) {
            // 宏名称
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + j*4, src[j]) != HAL_OK) {
                HAL_FLASH_Lock();
                return PARAM_ERROR_WRITE_FAILED;
            }
        }
    }

    HAL_FLASH_Lock();
    return PARAM_OK;
}
/*
Flash �洢��? ��һ�ַ���ʧ�Դ洢�������ϵ�����ݲ��ᶪʧ������ɾ������ҲҪ��������ȫ������ 
�����Flash���߼��ϵ�Ŀ����ʣ�
����ʵ��ʱ�����ǻ��õ�����ĵ�ַ�;����Ӳ���������������磺
�� STM32 �У��� HAL ��� HAL_FLASH_Program()��HAL_FLASHEx_Erase()�Ⱥ�����
������Ŀ���ַ������֮ǰ����ĺ꣬���磺#define PARAM_FLASH_ADDR      0x08100000 


*/

 


static const SystemParams DEFAULT_PARAMS = {

// ����ṹ��SystemParams DEFAULT_PARAMS�����洢Ĭ�ϲ���ֵ
//����ֱ��ʹ��int xxx = xxx���и�ֵ�������Ƚ�һЩ����������ṹ�壬���涨�ýṹ��Ϊ�����ṹ��
/*��Ҫ����Ϊ��ֵ����1.���ݷ�ɢ���ֿ�����������ǿ��������޹�����
2.���������������ܰ����ǵ���һ�����崫�ݡ����桢�Ƚϡ�
3.���׳�����������ʱ���ܴ������˳�򣬻���©��ĳ��������
4.�޷�������У�飺������ħ��һ������֤�������ÿ����Ч�ԡ� 
�ýṹ��ĺô���
1.�߼����飺����ص����������һ���ṹ�����������
2.ͳһ�ӿڣ��������Խ���һ�� SystemParams *������������һ����������
3.����洢/���䣺����һ�� memcpy�� Flash��EEPROM����ͨ�����緢�͡�
4.����չ���Ժ�Ҫ���²��������� white_balance����ֻ���ڽṹ���һ���ֶΣ����÷����øĺ���ǩ����
5.��Ԫ���ݣ�ħ����CRC ���ֶ��ǡ������ԡ��ģ����������޷���Ȼ������Щ��
*/	


    .magic = PARAM_MAGIC,		 // ħ��������ʶ���Ƿ�����Ч����
/*
�����ݽṹ��ͷ���õ�һ���̶���ֵ��������ʶ���ݵ���Ч�Ի��ʽ��
�����magic�ֶ������ж���������ǲ�����Ч�� SystemParams���á�
�����ȡʱ������ magic == PARAM_MAGIC���������Ⱦ���Ϊ�����𻵻�δ��ʼ����
*/
    .exposure_us = 20000,    // 20ms
    .gain = 16,
    .output_rate_hz = 5,
    .crc = 0  // ���ڳ�ʼ��ʱ����
    
/*
�ṹ���Աʹ��.xxxx��ʽ��C99֧�ָø�ʽ�����Ը��ṹ���ĳ����Ա��ֵ��˳����Բ�������˳��
��߿ɶ��ԣ�һ�۾��ܿ���ÿ��ֵ�ĺ���
SystemParams p = {
    .magic = PARAM_MAGIC,
    .exposure_us = 20000,
    ...
};
�ȼ���
SystemParams p = { PARAM_MAGIC, 20000, 16, 5, 0 };
ָ����ʼ��������������������ṹ���Ա˳��ı䣬������Ȼ��ȷ��

*/
};

ParamStatus param_init(SystemParams* params) {
    if (!params) return PARAM_ERROR_INVALID_PARAM; //��ָ���� 

    // ���Դ�Flash��ȡ
    ParamStatus status = read_params(params);
    if (status == PARAM_OK) {
        // ��֤ħ����CRC
        if (params->magic != PARAM_MAGIC) {
            status = PARAM_ERROR_MAGIC_MISMATCH;
        } else {
            // ����CRC���ų�crc�ֶ�����
            // offsetof(SystemParams, crc) �õ�crc�ֶ��ڽṹ���е�ƫ��
            uint16_t crc_calc = crc16((uint8_t*)params, offsetof(SystemParams, crc));
            if (crc_calc != params->crc) {
                status = PARAM_ERROR_CRC_MISMATCH;
            }
        }
    }

    // �����ȡʧ�ܻ�У��ʧ�ܣ�ʹ��Ĭ�ϲ���
    if (status != PARAM_OK) {
        param_set_default(params);
        // ���Խ�Ĭ�ϲ���д��Flash
        param_save(params);//���浽flash 
    }

    return PARAM_OK;
}

/*
ִ������: 
�������ָ���Ƿ���Ч
���Դ�Flash��ȡ����
�����ȡ�ɹ�����֤ħ����CRC
�����֤ʧ�ܣ���Ĭ�ϲ���������
���سɹ�״̬

*/

ParamStatus param_save(const SystemParams* params) {
    if (!params) return PARAM_ERROR_INVALID_PARAM;
    return write_params(params);//����д�뺯�� 
}

ParamStatus param_update(SystemParams* params, uint8_t id, uint32_t value) {
    if (!params) return PARAM_ERROR_INVALID_PARAM;

    // ����ID���¶�Ӧ�ֶΣ�ʾ��������չ��
    switch (id) {
        case 0: // �ع�ʱ��
            if (value >= 1000 && value <= 500000) // ��Χ���
                params->exposure_us = (uint16_t)value;
            else
                return PARAM_ERROR_INVALID_PARAM;
            break;
        case 1: // ����
            if (value <= 255)
                params->gain = (uint8_t)value;
            else
                return PARAM_ERROR_INVALID_PARAM;
            break;
        case 2: // ���Ƶ��
            if (value == 1 || value == 2 || value == 5 || value == 10)
                params->output_rate_hz = (uint8_t)value;
            else
                return PARAM_ERROR_INVALID_PARAM;
            break;
        default:
            return PARAM_ERROR_INVALID_PARAM;
    }

    // ����CRC
    params->crc = crc16((uint8_t*)params, offsetof(SystemParams, crc));
    return PARAM_OK;
}

void param_set_default(SystemParams* params) {
    if (!params) return;
    memcpy(params, &DEFAULT_PARAMS, sizeof(SystemParams));
    params->crc = crc16((uint8_t*)params, offsetof(SystemParams, crc));
}

/* ==================== �ڲ�ʵ�� ==================== */

// ��CRC16 (CCITT)
static uint16_t crc16(const uint8_t* data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// ��Flash��ȡ���������ݱ��ݣ����������
static ParamStatus read_params(SystemParams* params) {
    uint32_t base = PARAM_FLASH_ADDR;
    SystemParams copies[PARAM_BACKUP_COUNT];
    uint32_t errors[PARAM_BACKUP_COUNT] = {0};

    // 读取三个备份
    for (int i = 0; i < PARAM_BACKUP_COUNT; ++i) {
        // 直接从地址读取
        memcpy(&copies[i], (const void*)(base + i * PARAM_SECTOR_SIZE), sizeof(SystemParams));
        // У��ÿ�����ݵ�CRC
        uint16_t crc_calc = crc16((uint8_t*)&copies[i], offsetof(SystemParams, crc));
        if (copies[i].magic != PARAM_MAGIC || crc_calc != copies[i].crc) {
            errors[i] = 1;
        }
    }

    // ��û�д���ı���
    int good_idx = -1;
    for (int i = 0; i < PARAM_BACKUP_COUNT; ++i) {
        if (!errors[i]) {
            good_idx = i;
            break;
        }
    }
    if (good_idx >= 0) {
        *params = copies[good_idx];
        return PARAM_OK;
    }

    // ���б��ݶ���
    return PARAM_ERROR_READ_FAILED;
}
