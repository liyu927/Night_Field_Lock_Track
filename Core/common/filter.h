#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/* 一阶低通滤波器的运行状态，每一路 ADC 数据各使用一个实例。 */
typedef struct
{
    uint16_t value;          /* 上一次滤波输出，同时作为本次计算的历史值。 */
    uint8_t initialized;     /* 0 表示尚未接收首个样本，非 0 表示已完成初始化。 */
} LowPassFilter_t;

/* 三点中值滤波器的运行状态，每一路 ADC 数据各使用一个实例。 */
typedef struct
{
    uint16_t samples[3];     /* 连续保存的三个 ADC 样本，数组下标 0 为最早、2 为最新。 */
    uint8_t initialized;     /* 0 表示尚未接收首个样本，非 0 表示已完成初始化。 */
} Median3Filter_t;

/* 标量卡尔曼滤波器的运行状态，适用于单个 ADC 通道。 */
typedef struct
{
    float estimate;          /* 当前时刻的最优估计值。 */
    float covariance;        /* 当前估计误差协方差，数值越大代表估计越不确定。 */
    float process_noise;     /* 过程噪声 Q，描述真实光照在相邻采样间的变化程度。 */
    float measurement_noise; /* 测量噪声 R，描述 ADC 采样抖动程度。 */
    uint8_t initialized;     /* 0 表示首个测量值尚未写入估计值。 */
} KalmanFilter_t;

/**
 * @brief 对单个 ADC 样本执行一阶低通滤波。
 * @param filter 滤波器状态，首次调用时会自动以输入值初始化。
 * @param input 本次 ADC 原始输入值。
 * @param shift 低通系数的右移位数，3 表示系数为 1/8。
 * @return 本次滤波后的 ADC 值。
 */
uint16_t Filter_LowPass(LowPassFilter_t *filter, uint16_t input, uint8_t shift);

/**
 * @brief 对单个测量值执行标量卡尔曼滤波。
 * @param filter 包含 Q、R 和内部估计状态的滤波器。
 * @param measurement 本次测量值。
 * @return 本次更新后的最优估计值。
 */
float Filter_Kalman(KalmanFilter_t *filter, float measurement);

/**
 * @brief 返回三个样本中的中位数，用于消除偶发的 ADC 尖峰。
 * @param sample1 第一个 ADC 样本。
 * @param sample2 第二个 ADC 样本。
 * @param sample3 第三个 ADC 样本。
 * @return 三个样本按数值排序后的中间值。
 */
uint16_t Filter_Median3(uint16_t sample1, uint16_t sample2, uint16_t sample3);

/**
 * @brief 将最新 ADC 样本写入三点历史记录并返回中值滤波结果。
 * @param filter 三点中值滤波器状态，首次调用时会用输入值填满三个历史样本。
 * @param input 本次 ADC 原始输入值。
 * @return 最近三个 ADC 样本的中位数。
 */
uint16_t Filter_Median3Update(Median3Filter_t *filter, uint16_t input);

#endif
