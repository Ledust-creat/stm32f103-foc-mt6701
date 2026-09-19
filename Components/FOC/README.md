# FOC 可移植内核（第一阶段）

本目录不依赖 STM32 HAL，目标是让坐标变换和后续控制器能在 STM32、TI、NXP 或主机单元测试中复用。

```text
ADC/编码器/PWM BSP  <->  foc_port.h  <->  FOC 算法核  <->  APP 模式与目标
```

## 已提供

| 文件 | 职责 | 不允许依赖 |
| --- | --- | --- |
| `foc_math.*` | Clarke、Park 及其逆变换 | HAL、定时器、全局电机状态 |
| `foc_port.h` | 电流、角度、PWM 的最小平台契约 | 具体 MCU/HAL |

## 坐标约定

输入相序固定为 U/V/W，三相平衡时 `Iw = -(Iu + Iv)`；`electrical_angle_rad` 是弧度制电角度，定义为 `pole_pairs * (mechanical_angle - electrical_zero)`。若编码器或相序方向改变，必须在板级配置中统一，而不是偷偷修改 Park 公式。

## 下一阶段

为 `foc_math` 添加主机单元测试，验证 Park 后再反 Park 的误差；测试通过后才加入 SVPWM 和 STM32F103 的 `foc_port_stm32f1_hal.c`。
