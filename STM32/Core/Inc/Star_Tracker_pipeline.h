/**
 * @file star_tracker.h
 * @brief ������������ˮ�߽ӿ�
 * 
 * ����������ȡ����ͼʶ���Ǳ��ѯ����̬�����ģ�顣
 * ����������ȡ����ͼʶ����ʵ�֣���ģ�鸺����á�
 */

#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "Davenport_q.h"
#include "pinhole_camera.h"
#include "centroid.h"
#include "param.h"
#include "star_detector.h"

#include "config.h"

#include <stdio.h>
#include <string.h>



#ifdef __cplusplus
extern "C" {
#endif

/* �����루ϸ���� */
typedef enum {
    STAR_TRACKER_SUCCESS = 0,
    STAR_TRACKER_ERROR_INVALID_PARAM = -1,
    STAR_TRACKER_ERROR_NOT_INITIALIZED = -2,
    STAR_TRACKER_ERROR_IMAGE_SIZE_MISMATCH = -3,
    STAR_TRACKER_ERROR_STAR_COUNT_EXCEEDED = -4,
    STAR_TRACKER_ERROR_NO_STARS = -5,
    STAR_TRACKER_ERROR_CENTROID_FAILED = -6,
    STAR_TRACKER_ERROR_IDENTIFY_FAILED = -7,
    STAR_TRACKER_ERROR_CATALOG_FAILED = -8,
    STAR_TRACKER_ERROR_SOLVER_FAILED = -9
} StarTrackerStatus;

/* ���ýṹ */
typedef struct {
    int image_width;
    int image_height;
    int max_detected_stars;      // �������
    PinholeCamera camera;         // ����ڲ�
    StarDetectorConfig detector_cfg;   // �ǵ��� 
} StarTrackerConfig;

/* ����ˮ��ʵ������͸��ָ�룩 */
typedef struct StarTrackerInstance StarTrackerInstance;

/**
 * @brief ��������ʼ����ˮ��ʵ��
 * @param config ���ò���
 * @return ʵ��ָ�룬ʧ�ܷ���NULL
 */
StarTrackerInstance* star_tracker_create(const StarTrackerConfig* config);

/**
 * @brief ������ˮ��ʵ��
 */
void star_tracker_destroy(StarTrackerInstance* inst);

/**
 * @brief ����һ֡ͼ��
 * @param inst         ʵ��
 * @param image        ͼ�����ݣ��Ҷȣ�16λ��
 * @param star_pixels  �ǵ���������飨ÿ��Ԫ��[x,y]��
 * @param star_count   �ǵ�����
 * @param quat_out     �����Ԫ�� [w,x,y,z]���ڴ��ɵ����߷��䣩
 * @return StarTrackerStatus
 */
StarTrackerStatus star_tracker_process_frame(
    StarTrackerInstance* inst,
    const uint16_t* image,
    int image_width,
    int image_height,
    float quat_out[4],
	int* out_detected,   // 【新增】输出检测到的星数
	int* out_matched     // 【新增】输出匹配成功的星数
);

#ifdef __cplusplus
}
#endif

#endif // STAR_TRACKER_H
