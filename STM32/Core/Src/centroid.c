/**
 * @file centroid.c
 * @brief 质心提取模块实现（平方加权质心算法）
 * 
 * 算法步骤：
 * 1. 估计背景和噪声标准差（取窗口边缘像素）
 * 2. 计算阈值 = 背景 + threshold_sigma * 噪声
 * 3. 对超过阈值的像素计算平方加权质心
 * 4. 输出亚像素坐标
 */

#include "centroid.h"
#include <math.h>
#include <string.h>
#include <float.h>

// 内部函数：估计背景和噪声（使用窗口四周的像素）
/*当我们进行质心提取时，我们本质上是在选取那些自身数值显著高于周围像素值的点。 
因此在进行质心提取之前我们需要首先计算背景噪声的值。这样我们就得到了参考值，只要比参考值更亮的像素，且数值超过规定阈值，我们就认为是星点 
*/ 
static void estimate_background_and_noise/*这里的static在函数前是作用域限制，代表本函数只在当前文件中被调用，
因此我们可以注意到，pipeline中需要调用其它文件的对应函数不添加static，而其余所有函数都添加static
该程序中， estimate_background_and_noise，CentroidStatus square_weighted_centroid两个函数添加static限制，
entroid_config_default和CentroidStatus centroid_compute需要被pipeline调用，所以不添加static*/
(const uint16_t* roi,int width, int height,float* background,float* noise_std) {
/*这里传输的变量有：
1.ROI，即region of interest，星点对应的一个小邻域像素，可以减轻计算量，我们选择*rio来传递邻域的指针， 
2. int width, int height图像的长度和宽
3.float* background背景变量的地址指针，这样该函数计算出背景值后，直接将该值修改，然后其它程序根据地址读取内存中的值即可 
4.float* noise_std噪声标准，传入指针，直接修改值，同理 
*/
    int count = 0;
    double sum = 0.0, sum_sq = 0.0;
//count用来循环计数，sum和sum_sq分别代表求和和平方和,这里计算平方和是为了后面做方差方便计算 

/*对于背景噪声的估计，思路是
当图像传入时，star_identification已经帮忙识别出了星点并给出了亮点附近的一个邻域（窗口，即ROI） 
而在ROI中，我们需要更仔细地提取亮点的像素，并尽量排除噪声和背景干扰。
我们取ROI窗格四周像素，因为一般认为星点出现在图像中间的概率更大，而ROI边缘是漆黑的背景 
范围是：ROI的第一行、最后一行、第一列、最后一列（但是注意四个角会在第一行和第一列取到，因此要避免重复计算四个角的点）*/
    // 第一行 (x取0到wideth-1，y=0)
    for (int x = 0; x < width; ++x) {
        float val = (float)roi[x];
    //定义value（val）为存储这些值的变量，（float）为强制类型转换
	/*这里使用一维数组roi[x]来代表二维图像，因为我们只是选取几个边缘点，不需要二维数组增加计算量
	使用一维数组表示二维的逻辑如下：
	二维视角（逻辑）:
	(0,0) (1,0) (2,0) (3,0) (4,0)   ← 第0行
	(0,1) (1,1) (2,1) (3,1) (4,1)   ← 第1行
	(0,2) (1,2) (亮点) (3,2) (4,2)   ← 第2行
	(0,3) (1,3) (2,3) (3,3) (4,3)   ← 第3行
	(0,4) (1,4) (2,4) (3,4) (4,4)   ← 第4行

	一维视角（内存）:
	roi[0]  = (0,0)
	roi[1]  = (1,0)
	roi[2]  = (2,0)
	roi[3]  = (3,0)
	roi[4]  = (4,0)
	roi[5]  = (0,1)
	roi[6]  = (1,1)
	...
	roi[24] = (4,4)
	
	公式:
	对于任意 (x, y)其中x表示列，y表示行，则有：
	index = y * width + x;
	比如最后一个像素，就是roi[index]= roi[24]=roi[y*width+x]=roi[4*5+4]=roi[24] 
	pixel_value = roi[index];
	*/ 
        sum += val;
        sum_sq += val * val;
        count++;
/*sum：所有背景像素值的和
sum_sq：所有背景像素值的平方和
count：背景像素个数
*/
    }
    // 最后一行 (y=height-1)，同理 
    if (height > 1) {
        int row_start = (height - 1) * width;
        for (int x = 0; x < width; ++x) {
            float val = (float)roi[row_start + x];
            sum += val;
            sum_sq += val * val;
            count++;
        }
    }
    // 第一列 (x=0) 和最后一列 (x=width-1)，跳过已计入的角点，同理 
    for (int y = 1; y < height - 1; ++y) {
        int row = y * width;
        // 第一列
        float val_left = (float)roi[row];
        sum += val_left;
        sum_sq += val_left * val_left;
        count++;
        // 最后一列
        float val_right = (float)roi[row + width - 1];
        sum += val_right;
        sum_sq += val_right * val_right;
        count++;
    }

    if (count > 0) {
        *background = (float)(sum / count);
        // 计算标准差： sqrt( (sum(x^2) - n*mean^2) / (n-1) )
        double mean = sum / count;
        double variance = (sum_sq - count * mean * mean) / (count - 1);
        *noise_std = (float)(sqrt(variance > 0 ? variance : 0));
    } else {
        // 防御性编程，如果count等于0，即图像传输或解析错误，此时像素数量为0，则强制背景和噪声为float，值为0.0 
        *background = 0.0f;
        *noise_std = 0.0f;
    }
}

// 平方加权质心计算（内部，不检查参数）
/*
当我们计算出RIO窗口的背景，噪声后，我们就需要计算：排除背景后，当前像素的最亮点处于哪里？（类似寻找不规则物体的质心）

假设有一个3x3的光斑，排除噪声后，数字代表亮度值：
10   20   30
40   255  50
30   45   25

质心计算会考虑：
1. 越亮的像素权重越大
2. 这里是平方加权（val * val），让亮像素的影响更显著 

质心公式：

         ∑(x * weight)    ∑(x * val2)
cx = ----------------- = ------------
           ∑weight           ∑val2

cy同理 

示例：一行像素
位置 x:  0    1    2    3    4
亮度 val: 0   10   50   20   0
平方 w:  0   100  2500  400  0

加权和 sum_x = 1*100 + 2*2500 + 3*400 = 100 + 5000 + 1200 = 6300
总权重 total = 100 + 2500 + 400 = 3000
质心 cx = 6300/3000 = 2.1
结果偏向亮度最高的位置（x=2）

*/
static CentroidStatus square_weighted_centroid
(const uint16_t* roi,int width, int height,float threshold,float* cx, float* cy) {
	//这本质上是一个累加器，使用double确保精度
    double total_weight = 0.0;   // 所有像素亮度的平方和 
    double sum_x = 0.0, sum_y = 0.0;// 加权位置和
    int count = 0;

    for (int y = 0; y < height; ++y) {//遍历每个像素 
        for (int x = 0; x < width; ++x) {
            float val = (float)roi[y * width + x];
            if (val > threshold) { // 只处理超过阈值的像素（避免噪声影响）
                // 平方加权
                float w = val * val;// 权重 = 亮度的平方
                total_weight += w;// 累加总权重
                sum_x += x * w; // x坐标加权和
                sum_y += y * w;// y坐标加权和
                count++;
            }
        }
    }
 // 错误检查：没有有效像素或总权重太小
    if (count == 0 || total_weight < 1e-6f) {
        return CENTROID_ERROR_NO_VALID_PIXELS;
    }
  // 计算质心坐标 = 加权和 / 总权重
    *cx = (float)(sum_x / total_weight); // 转换为float存入cx指向的位置
    *cy = (float)(sum_y / total_weight);// 转换为float存入cy指向的位置
    return CENTROID_SUCCESS;
}

// 公开 API
void centroid_config_default(CentroidConfig* config) {
    if (config != NULL) {// 安全检查：防止空指针
        config->method = CENTROID_SQUARE_WEIGHTED;// 默认使用平方加权
        config->threshold_sigma = 3.0f;   
		// 3倍噪声标准差,这是满足统计学的3σ原则（中心点左右各3个σ，分布曲线面积占到99.73%） 
        config->window_size = 7;// 默认检索7x7 窗口，作为常见光斑大小的估计 
    }
}

CentroidStatus centroid_compute(
    const CentroidConfig* config,//输入配置参数 
    const uint16_t* roi,//输入图像数据 
    int roi_width, int roi_height,//输入图像尺寸 
    float* cx, float* cy)//输出计算结果 
{
    if (!config || !roi || !cx || !cy ||//检查指针 
        roi_width <= 0 || roi_height <= 0 ||
        roi_width != config->window_size || roi_height != config->window_size) {
        return CENTROID_ERROR_INVALID_PARAM;
        /*
		当逻辑运算符作用于指针时，如果指针为空，那么!conifg为true，如果指针不为NULL，则!config为false
		逻辑且为&&逻辑或为||
		上文涵义即是说
		只要其中任意一个指针为 NULL，整个表达式的结果就是 true，
		于是进入 if分支执行相应的错误处理代码（比如打印日志、返回错误码等） 
		*/
    }

    // 目前仅支持平方加权算法
    if (config->method != CENTROID_SQUARE_WEIGHTED) {
        return CENTROID_ERROR_INVALID_PARAM;
    }

    // 估计背景和噪声
    float background, noise_std;
    estimate_background_and_noise(roi, roi_width, roi_height,
                                  &background, &noise_std);

    // 计算阈值
    float threshold = background + config->threshold_sigma * noise_std;
    if (threshold < 1.0f) threshold = 1.0f;  // 至少为1，避免无像素

    // 调用平方加权质心
    CentroidStatus status = square_weighted_centroid(roi,
                                                     roi_width, roi_height,
                                                     threshold,
                                                     cx, cy);
    return status;
}
