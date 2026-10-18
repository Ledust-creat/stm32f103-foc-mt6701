# 来源与第三方组件说明

本项目的 FOC 控制逻辑、控制接口、串口试验应用及个人辅助代码由作者独立编写。下列教程与参考工程用于学习原理和实现思路，列出链接是致谢和学习路径说明，不表示本项目复制其源码或在其工程上二次开发。

STM32 HAL、CMSIS、CMSIS-DSP 等第三方组件及工具生成代码与个人实现分开标注；本文不替代其原始许可证，也不为整个工程新增统一授权。

| 内容 | 来源线索 / 本地许可文件 | 当前处理 |
| --- | --- | --- |
| FOC 学习教程 | https://blog.csdn.net/qq570437459/article/details/142188261 | 原理及实现思路的学习参考，不作为源码复制来源 |
| FOC 参考工程 | https://gitee.com/best_pureer/stm32_foc | 学习参考，不作为本项目二次开发的归属声明 |
| STM32 HAL | `Drivers/STM32F1xx_HAL_Driver/LICENSE.txt` 及各源文件头部 | 保留原始版权、许可文本和适用条件 |
| CMSIS / STM32 设备支持 | `Drivers/CMSIS/LICENSE.txt`、`Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt` | 保留组件各自许可，不把其许可自动扩展到个人代码或其他来源 |
| CMSIS-DSP | `Middlewares/Third_Party/ARM/DSP/LICENSE.txt` 及相关库、头文件声明 | 同时核对实际使用的 `Middlewares/ST/ARM/DSP/`，保留配套许可和版权 |

个人实现包括控制接口、三环联调、串口位置试验、纯 P 限幅偏差修复等，具体以源码和历史记录为准。独立实现与所依赖的厂商库、算法库及生成代码应分别说明，不将第三方组件重新署名为个人作品。

上位机联调曾基于 [KINGSTON-115/llm-pid-tuner](https://github.com/KINGSTON-115/llm-pid-tuner) 扩展，该上位机当前不包含在此 MCU 仓库中，其许可不能自动用于本仓库其他文件。
