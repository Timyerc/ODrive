# Patch 应用情况

`odrive-keep-armed-on-error.patch` 针对的固件版本与当前工程不同，未能完整应用。
此前认为核心功能已经实现的结论不正确。

- `axis.cpp` 中的 watchdog 超时检查已被绕过。
- `low_level.cpp` 中部分制动时序、占空比和母线电流检查已被修改。
- `ODRIVE_KEEP_ARMED_ON_ERROR` 宏目前没有接入停机逻辑，设置为 0 或 1
  均不能切换出错停机行为。
- `last_pos_estimate_local_` 已声明，但未接入回零逻辑。
- Controller、Motor、Encoder、Main、SensorlessEstimator 和 Thermistor 的
  补丁改动尚未完整移植。

当前版本用 `brake_resistance = 0` 禁用制动电阻。原补丁使用的
`DEFAULT_BRAKE_RESISTANCE` 在本版本中不存在，错误移植的默认值回退已移除。

这些改动会绕过部分保护，且未证明固件会在出错时保持电机使能。
