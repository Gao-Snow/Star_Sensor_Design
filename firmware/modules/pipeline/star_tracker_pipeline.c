/**
 * @file star_tracker.c
 * @brief 星敏感器主流水线实现
 */

#include "star_tracker.h"
#include "catalog.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 假设的质心提取和星图识别模块接口（需要实际实现）
// 此处仅为声明，实际使用时需包含对应头文件
StarTrackerStatus centroid_extract(
    const uint16_t* image, int img_w, int img_h,
    int star_x, int star_y,
    float* cx, float* cy
);

StarTrackerStatus star_identify(
    const float centroids[][2],
    int star_count,
    int star_ids[]
);

// 流水线实例结构（隐藏内部细节）
struct StarTrackerInstance {
    StarTrackerConfig config;
    CentroidConfig centroid_cfg;
    QuestConfig quest_cfg;
    StarDetector* detector;
    StarIdentifier* identifier;
    SystemParams params;
    CommandCallback cmd_cb;
    // 可添加其他状态
};

StarTrackerInstance* star_tracker_create(const StarTrackerConfig* config) {
    if (!config || config->max_detected_stars <= 0 ||
        config->image_width <= 0 || config->image_height <= 0) {
        return NULL;
    // 初始化参数模块
    ParamStatus pstat = param_init(&inst->params);
    if (pstat != PARAM_OK) {
        // 使用默认参数（已由param_init设置）
    
    // 注册指令回调（将comm的回调指向本模块的处理函数）
    comm_register_callback(star_tracker_command_handler);
    inst->cmd_cb = star_tracker_command_handler;  // 保存

    return inst;
    }

    StarTrackerInstance* inst = (StarTrackerInstance*)malloc(sizeof(StarTrackerInstance));
    if (!inst) return NULL;

    memcpy(&inst->config, config, sizeof(StarTrackerConfig));
	
	//初始化质心提取配置 
	centroid_config_default(&inst->centroid_cfg);
    // 可根据需要从 config 中读取参数覆盖默认值
    // 例如：inst->centroid_cfg.window_size = config->centroid_window_size;
	
    // 初始化 QUEST 配置（可根据需要从 config 读取）
    inst->quest_cfg.max_iterations = 100;
    inst->quest_cfg.convergence_thresh = 1e-6f;

    // 初始化星表（若需要）
    catalog_init();

    return inst;
}

//指令处理函数
// 静态函数，处理接收到的指令
static void star_tracker_command_handler(uint8_t cmd, uint8_t* data, uint16_t len) {
    StarTrackerInstance* inst = get_global_instance(); // 假设有全局实例指针

    switch (cmd) {
        case FRAME_TYPE_CMD_RESET:
            // 软件复位（可通过NVIC）
            NVIC_SystemReset();
            break;

        case FRAME_TYPE_CMD_MODE:
            if (len >= 1) {
                uint8_t mode = data[0];
                // 切换模式（需实现mode_mgr）
                // mode_mgr_set_mode(mode);
                comm_send_ack(0);
            } else {
                comm_send_ack(1);
            }
            break;

        case FRAME_TYPE_CMD_SET_PARAM:
            if (len >= 3) { // 参数ID(1) + 值(2或4)
                uint8_t param_id = data[0];
                uint32_t value;
                if (len == 3) {
                    value = data[1] | (data[2] << 8);
                } else if (len == 5) {
                    value = data[1] | (data[2]<<8) | (data[3]<<16) | (data[4]<<24);
                } else {
                    comm_send_ack(2); // 长度错误
                    break;
                }
                // 更新参数
                ParamStatus ps = param_update(&inst->params, param_id, value);
                if (ps == PARAM_OK) {
                    // 保存到Flash
                    param_save(&inst->params);
                    comm_send_ack(0);
                } else {
                    comm_send_ack(3); // 参数无效
                }
            }
            break;

        case FRAME_TYPE_CMD_UPGRADE:
            // 固件升级处理（略）
            break;

        default:
            comm_send_ack(0xFF); // 未知命令
            break;
    }
}
 

void star_tracker_destroy(StarTrackerInstance* inst) {
    if (inst) free(inst);
}

StarTrackerStatus star_tracker_process_frame(
    StarTrackerInstance* inst,
    const uint16_t* image,
    const int star_pixels[][2],
    int star_count,
    float quat_out[4])
{
    if (!inst || !image || !star_pixels || !quat_out) {
        return STAR_TRACKER_ERROR_INVALID_PARAM;
    }
    if (star_count > inst->config.max_detected_stars) {
        return STAR_TRACKER_ERROR_STAR_COUNT_EXCEEDED;
    }
    if (star_count == 0) {
        return STAR_TRACKER_ERROR_NO_STARS;
    }

    // 临时缓冲区（使用VLA，但确保最大星数不太大）
    float centroids[star_count][2];
    int star_ids[star_count];
    QuestVector3 body_vec[star_count];
    QuestVector3 ref_vec[star_count];

    // 1. 质心提取
	float centroids[star_count][2];
	for (int i = 0; i < star_count; ++i) {
    // 提取 ROI（同前）
    	int win = inst->centroid_cfg.window_size;
    	int half = win / 2;
    	int x0 = star_pixels[i][0] - half;
    	int y0 = star_pixels[i][1] - half;
    // ... 边界检查 ...
    uint16_t roi[81];
    for (int r = 0; r < win; ++r)
        for (int c = 0; c < win; ++c)
            roi[r*win + c] = image[(y0+r)*img_w + (x0+c)];

    float cx_rel, cy_rel;
    CentroidStatus cs = centroid_compute(&inst->centroid_cfg, roi, win, win, &cx_rel, &cy_rel);
    if (cs != CENTROID_SUCCESS) {
        return STAR_TRACKER_ERROR_CENTROID_FAILED;
    }
    centroids[i][0] = x0 + cx_rel;
    centroids[i][1] = y0 + cy_rel;
	}

    // 2. 星图识别
    StarTrackerStatus status = star_identify(centroids, star_count, star_ids);
    if (status != STAR_TRACKER_SUCCESS) {
        return status;
    }

    // 3. 将质心坐标转换为本体系单位向量
    for (int i = 0; i < star_count; ++i) {
        float vec[3];
        if (!camera_pixel_to_vector(&inst->config.camera,
                                    centroids[i][0], centroids[i][1],
                                    vec)) {
            return STAR_TRACKER_ERROR_CENTROID_FAILED;  // 转换失败
        }
        body_vec[i].x = vec[0];
        body_vec[i].y = vec[1];
        body_vec[i].z = vec[2];
    }

    // 4. 查询星表获得参考系向量
    for (int i = 0; i < star_count; ++i) {
        float vec[3];
        CatalogStatus cs = catalog_get_vector(star_ids[i], vec);
        if (cs != CATALOG_SUCCESS) {
            return STAR_TRACKER_ERROR_CATALOG_FAILED;
        }
        ref_vec[i].x = vec[0];
        ref_vec[i].y = vec[1];
        ref_vec[i].z = vec[2];
    }

    // 5. 调用 QUEST 解算姿态
    QuestResult qres;
    QuestStatus qs = quest_solve(
        &inst->quest_cfg,
        body_vec, ref_vec,
        NULL,  // 权重为NULL表示等权重
        star_count,
        &qres
    );
    if (qs != QUEST_SUCCESS) {
        return STAR_TRACKER_ERROR_SOLVER_FAILED;
    }

    // 输出四元数
    quat_out[0] = qres.quat.w;
    quat_out[1] = qres.quat.x;
    quat_out[2] = qres.quat.y;
    quat_out[3] = qres.quat.z;

    return STAR_TRACKER_SUCCESS;
}

//在主循环中，处理完一帧图像后，调用 comm_send_attitude 发送姿态
void star_tracker_loop(StarTrackerInstance* inst) {
    while (1) {
        // 等待图像采集完成（略）
        // 处理一帧
        float quat[4];
        StarTrackerStatus status = star_tracker_process_frame(inst, image, width, height, quat);
        if (status == STAR_TRACKER_SUCCESS) {
            // 获取时间戳
            uint32_t timestamp = HAL_GetTick();
            uint8_t state = 0; // 状态字
            comm_send_attitude(quat, timestamp, state);
        }

        // 发送遥测（可降低频率，如每10帧一次）
        static uint32_t last_telemetry = 0;
        if (HAL_GetTick() - last_telemetry > 1000) { // 1Hz
            int16_t temp = read_temperature(); // 需实现
            uint16_t volt = read_voltage();
            uint8_t star_cnt = inst->last_star_count; // 需记录
            uint8_t mode = 1; // 当前模式
            uint8_t error = 0; // 错误码
            comm_send_telemetry(temp, volt, star_cnt, mode, error);
            last_telemetry = HAL_GetTick();
        }

        // 处理接收到的指令
        comm_process();

        // 喂狗
        HAL_IWDG_Refresh(&hiwdg);
    }
}
