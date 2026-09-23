#ifndef FOC_FILTER_H
#define FOC_FILTER_H

/* 一阶低通；alpha为本次输入权重，范围[0, 1]，上次输出由调用者保存。 */
float FocFilter_LowPass(float input, float last_output, float alpha);

#endif /* FOC_FILTER_H */
