#include "star_detector.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// --- 新增：使用静态全局内存替代 malloc，彻底告别内存崩溃 ---
#define MAX_IMAGE_PIXELS (320 * 240)
#define MAX_LABELS 1000

static uint16_t g_labels[MAX_IMAGE_PIXELS];
static int g_remap[MAX_LABELS];

typedef struct {
    int count;
    double sum_x;
    double sum_y;
} BlobStat;
static BlobStat g_stats[MAX_LABELS];
// --------------------------------------------------------

static uint16_t find(uint16_t* equiv, uint16_t x) {
    while (equiv[x] != x) {
        equiv[x] = equiv[equiv[x]];
        x = equiv[x];
    }
    return x;
}

static void union_sets(uint16_t* equiv, uint16_t a, uint16_t b) {
    uint16_t ra = find(equiv, a);
    uint16_t rb = find(equiv, b);
    if (ra != rb) equiv[rb] = ra;
}

static int label_image(const uint16_t* image, int width, int height,
                       float threshold, uint16_t* labels, int* label_count);

void star_detector_config_default(StarDetectorConfig* config) {
    if (config) {
        config->threshold_sigma = 4.0f;
        config->min_star_pixels = 1;
        config->max_stars = 50;
    }
}

bool star_detector_detect(const StarDetectorConfig* config,
                          const uint16_t* image,
                          int width, int height,
                          int star_pixels[][2],
                          int max_stars,
                          int* star_count) {
    if (!config || !image || !star_pixels || !star_count) return false;
    if (width <= 0 || height <= 0 || max_stars <= 0) return false;
    if (width * height > MAX_IMAGE_PIXELS) return false; // 安全检查

    *star_count = 0;

    // 1. 估计背景噪声
    double sum = 0.0, sum_sq = 0.0;
    int count = 0;
    for (int x = 0; x < width; ++x) {
        float val = (float)image[x];
        sum += val; sum_sq += val * val; count++;
    }
    if (height > 1) {
        int row_start = (height - 1) * width;
        for (int x = 0; x < width; ++x) {
            float val = (float)image[row_start + x];
            sum += val; sum_sq += val * val; count++;
        }
    }
    for (int y = 1; y < height - 1; ++y) {
        int row = y * width;
        float val_left = (float)image[row];
        float val_right = (float)image[row + width - 1];
        sum += val_left; sum_sq += val_left * val_left;
        sum += val_right; sum_sq += val_right * val_right;
        count += 2;
    }

    if (count == 0) return false;
    double mean = sum / count;
    double variance = (sum_sq - count * mean * mean) / (count - 1);
    float noise_std = (float)sqrt(variance > 0 ? variance : 0);
    float threshold = (float)mean + config->threshold_sigma * noise_std;
    if (threshold < 1.0f) threshold = 1.0f;

    // 2. 连通域标记 (替换掉 malloc)
    uint16_t* labels = g_labels;
    int label_count = 0;
    int num_labels = label_image(image, width, height, threshold, labels, &label_count);
    if (num_labels <= 0) {
        return true;
    }

    // 3. 统计 (替换掉 calloc)
    BlobStat* stats = g_stats;
    memset(stats, 0, num_labels * sizeof(BlobStat)); // 手动清零

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            uint16_t lab = labels[idx];
            if (lab > 0) {
                int l = lab - 1;
                stats[l].count++;
                stats[l].sum_x += x;
                stats[l].sum_y += y;
            }
        }
    }

    // 4. 提取质心
    int out_idx = 0;
    for (int i = 0; i < num_labels && out_idx < max_stars; ++i) {
        if (stats[i].count >= config->min_star_pixels) {
            float cx = (float)(stats[i].sum_x / stats[i].count);
            float cy = (float)(stats[i].sum_y / stats[i].count);
            star_pixels[out_idx][0] = (int)(cx + 0.5f);
            star_pixels[out_idx][1] = (int)(cy + 0.5f);
            out_idx++;
        }
    }
    *star_count = out_idx;
    return true;
}

static int label_image(const uint16_t* image, int width, int height,
                       float threshold, uint16_t* labels, int* label_count) {
    memset(labels, 0, width * height * sizeof(uint16_t));
    int next_label = 1;
    static uint16_t equiv[MAX_LABELS];
    for (int i = 0; i < MAX_LABELS; ++i) equiv[i] = i;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            if ((float)image[idx] <= threshold) continue;

            uint16_t neighbor_labels[4] = {0};
            int n_count = 0;
            if (x > 0 && labels[idx-1] != 0) neighbor_labels[n_count++] = labels[idx-1];
            if (y > 0 && labels[idx - width] != 0) neighbor_labels[n_count++] = labels[idx - width];
            if (x < width-1 && y > 0 && labels[idx - width + 1] != 0) neighbor_labels[n_count++] = labels[idx - width + 1];
            if (x > 0 && y > 0 && labels[idx - width - 1] != 0) neighbor_labels[n_count++] = labels[idx - width - 1];

            if (n_count == 0) {
            	if (next_label < MAX_LABELS) {
            	    labels[idx] = next_label++; // 安全范围内，正常打标签
            	}else {
                    labels[idx] = 0; // 超出1000个孤立光斑，直接视为噪声抛弃，绝不越界！
                }
            } else {
                uint16_t min_label = neighbor_labels[0];
                for (int i = 1; i < n_count; ++i) {
                    if (neighbor_labels[i] < min_label) min_label = neighbor_labels[i];
                }
                labels[idx] = min_label;
                for (int i = 0; i < n_count; ++i) {
                    if (neighbor_labels[i] != min_label) {
                        union_sets(equiv, min_label, neighbor_labels[i]);
                    }
                }
            }
        }
    }

    // 替换 calloc
    int* remap = g_remap;
    memset(remap, 0, next_label * sizeof(int));
    int new_label = 1;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            if (labels[idx] != 0) {
                uint16_t root = find(equiv, labels[idx]);
                if (remap[root] == 0) {
                    remap[root] = new_label++;
                }
                labels[idx] = remap[root];
            }
        }
    }
    *label_count = new_label - 1;
    return *label_count;
}
