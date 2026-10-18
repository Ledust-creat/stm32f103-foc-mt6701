# FOC 可移植接口草稿（尚未接入）

本目录保存早期平台接口设计，当前只有 `foc_port.h` 契约，尚未接入实际控制路径。Keil 工程实际使用 `Drivers/FOC/` 中的数学变换、SVPWM、PID、控制和硬件适配；不能将此目录视为已经完成的跨平台内核。

```text
ADC/编码器/PWM BSP  <->  foc_port.h  <->  FOC 算法核  <->  APP 模式与目标
```

## 已提供

| 文件 | 职责 | 不允许依赖 |
| --- | --- | --- |
| `foc_port.h` | 电流、角度、PWM 的最小平台契约 | 具体 MCU/HAL |

## 坐标约定

输入相序固定为 U/V/W，三相平衡时 `Iw = -(Iu + Iv)`；`electrical_angle_rad` 是弧度制电角度，定义为 `pole_pairs * (mechanical_angle - electrical_zero)`。若编码器或相序方向改变，必须在板级配置中统一，而不是偷偷修改 Park 公式。

## 下一阶段

若后续开展移植，可基于当前 `Drivers/FOC/` 中的实现整理平台依赖、添加主机数学测试，再实现并接入具体 MCU 的 Port；这是后续计划，不代表当前已有可直接用于 TI/NXP 的实现。当前运行工程已经包含 SVPWM，不需要为了匹配本草稿重新实现一遍。
