/**
 * @file param.h
 * @brief ��������ģ��ӿ�
 * ����ؼ������Ĵ洢�����غ͸��£�֧�ֵ��籣�档
 * �洢ϵͳ���в������ع⡢���桢���Ƶ�ʵȣ�������ʧ�洢����
 * ֧�����������CRCУ�顣
 */

#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* �����ṹ�壨�ɸ���������չ�� */
typedef struct {
    uint32_t magic;              /**< ħ�������ڱ�ʶ��Ч�� */
    uint16_t exposure_us;        /**< �ع�ʱ�䣨΢�룩 */
    uint8_t  gain;               /**< ���棨0-255�� */
    uint8_t  output_rate_hz;     /**< ���Ƶ�ʣ�Hz�� */
    uint16_t crc;                /**< CRC16У��ֵ */
} SystemParams;

/* ������ */
typedef enum {
    PARAM_OK = 0,
    PARAM_ERROR_INVALID_PARAM,
    PARAM_ERROR_READ_FAILED,
    PARAM_ERROR_WRITE_FAILED,
    PARAM_ERROR_CRC_MISMATCH,
    PARAM_ERROR_MAGIC_MISMATCH
} ParamStatus;

/**
 * @brief ��ʼ������ģ�飨��Flash���ز�����
 * @param params ��������ṹ��
 * @return ParamStatus
 */
ParamStatus param_init(SystemParams* params);

/**
 * @brief ���������Flash�����������ࣩ
 * @param params Ҫ����Ĳ���
 * @return ParamStatus
 */
ParamStatus param_save(const SystemParams* params);

/**
 * @brief ���µ���������������д��Flash��
 * @param params ��ǰ�����ṹ�壨�����޸ģ�
 * @param id      ����ID�����Զ���ö�٣�
 * @param value   ��ֵ����uint32_t���ݣ�
 * @return ParamStatus
 */
ParamStatus param_update(SystemParams* params, uint8_t id, uint32_t value);

/**
 * @brief �ָ�Ĭ�ϲ���
 * @param params ���Ĭ�ϲ���
 */
void param_set_default(SystemParams* params);

#ifdef __cplusplus
}
#endif

#endif // PARAM_H
