#include "filter.h"

/**
 * @brief 使用 y(k)=y(k-1)+(x(k)-y(k-1))/2^shift 计算一阶低通滤波。
 * @note 使用移位代替浮点除法，适合 STM32 上的高频 ADC 数据处理。
 */
uint16_t Filter_LowPass(LowPassFilter_t *filter, uint16_t input, uint8_t shift)
{
    int32_t difference; /* 输入值与上次滤波输出的有符号差值，避免相减时无符号下溢。 */

    /* 第一个样本直接作为初始输出，防止滤波器从零值缓慢爬升。 */
    if (filter->initialized == 0U)
    {
        filter->value = input;       /* 保存首个真实采样值。 */
        filter->initialized = 1U;    /* 标记后续调用可以进入正常滤波流程。 */
        return input;                /* 首次调用不引入额外误差。 */
    }

    difference = (int32_t)input - (int32_t)filter->value; /* 计算当前输入相对历史输出的偏差。 */
    filter->value = (uint16_t)((int32_t)filter->value + (difference >> shift)); /* 仅向输入值靠近 1/2^shift。 */
    return filter->value; /* 返回并保留本次结果，供下一次迭代使用。 */
}

/**
 * @brief 使用预测和测量更新步骤计算标量卡尔曼滤波结果。
 * @note 调用前需要由使用者设置 process_noise 和 measurement_noise。
 */
float Filter_Kalman(KalmanFilter_t *filter, float measurement)
{
    float gain; /* 卡尔曼增益，决定本次测量值对估计结果的修正比例。 */

    /* 首个测量值用于建立初始估计，避免估计值未定义。 */
    if (filter->initialized == 0U)
    {
        filter->estimate = measurement;  /* 以首个测量值作为初始真实值估计。 */
        filter->initialized = 1U;        /* 标记卡尔曼滤波器已经完成初始化。 */
        return measurement;              /* 首次调用直接返回测量值。 */
    }

    filter->covariance += filter->process_noise; /* 预测阶段：真实光照可能变化，因此增加不确定度。 */
    gain = filter->covariance / (filter->covariance + filter->measurement_noise); /* 根据 Q、R 计算本次测量的可信度。 */
    filter->estimate += gain * (measurement - filter->estimate); /* 用测量残差修正上一时刻的估计。 */
    filter->covariance *= (1.0f - gain); /* 更新阶段后，不确定度随新测量的引入而降低。 */
    return filter->estimate; /* 返回当前最优估计值。 */
}

/**
 * @brief 通过三次相邻比较求出三个无符号样本中的中位数(中值滤波)。
 * @note 中位数能剔除一个异常尖峰，且不需要保存跨周期状态。
 */
uint16_t Filter_Median3(uint16_t sample1, uint16_t sample2, uint16_t sample3)
{
    uint16_t temporary; /* 交换两个样本时使用的临时变量。 */

    /* 先确保 sample1 不大于 sample2。 */
    if (sample1 > sample2)
    {
        temporary = sample1; /* 暂存较大的第一个样本。 */
        sample1 = sample2;   /* 将较小的第二个样本放到前面。 */
        sample2 = temporary; /* 将较大的样本放到第二个位置。 */
    }
    /* 再确保 sample2 不大于 sample3。 */
    if (sample2 > sample3)
    {
        temporary = sample2; /* 暂存当前第二个样本。 */
        sample2 = sample3;   /* 将较小的第三个样本移到中间。 */
        sample3 = temporary; /* 将较大的样本移到最后。 */
    }
    /* 若最小值与中间值顺序仍颠倒，则第一个样本就是中位数。 */
    if (sample1 > sample2)
    {
        sample2 = sample1; /* 覆盖中间位置，使其保存三个值的中位数。 */
    }
    return sample2; /* 排序后的中间值即为滤波结果。 */
}

/**
 * @brief 更新三点历史样本，并对最近三个样本执行中值滤波。
 * @note 第一个样本会同时写入三个历史位置，避免启动阶段出现人为的零值尖峰。
 */
uint16_t Filter_Median3Update(Median3Filter_t *filter, uint16_t input)
{
    /* 首次采样时没有历史数据，因此以当前真实值填充全部三个位置。 */
    if (filter->initialized == 0U)
    {
        filter->samples[0] = input;  /* 最早样本初始化为当前值。 */
        filter->samples[1] = input;  /* 中间样本初始化为当前值。 */
        filter->samples[2] = input;  /* 最新样本初始化为当前值。 */
        filter->initialized = 1U;    /* 标记历史样本已可用于后续滚动更新。 */
        return input;                /* 三个样本相同，中值就是当前输入。 */
    }

    filter->samples[0] = filter->samples[1]; /* 丢弃最早样本，将原中间样本前移。 */
    filter->samples[1] = filter->samples[2]; /* 将原最新样本移到中间位置。 */
    filter->samples[2] = input;              /* 保存本周期最新 ADC 原始值。 */
    return Filter_Median3(filter->samples[0], filter->samples[1], filter->samples[2]); /* 返回最近三个样本的中值。 */
}
