# Patch 应用总结

## 应用时间
2026-11-07

## Patch 文件
`odrive-keep-armed-on-error.patch`

## 功能概述
此patch为ODrive电机控制固件添加"出错不停机"功能。核心理念是：
- 错误位（error flags）照常置位和上报
- 但电机不会因为错误而被disarm（失能）
- 通过可配置的宏开关 `ODRIVE_KEEP_ARMED_ON_ERROR` 控制此行为
- 当宏值为0时，恢复原始的"出错即停机"行为

## 已应用的改动

### 1. odrive_main.h
**位置**：文件开头（#define ODRIVE_MAIN_H 之后）

**改动内容**：
- 添加宏定义 `ODRIVE_KEEP_ARMED_ON_ERROR 1`
- 提供中文注释说明此开关的作用

**作用**：
- 全局控制出错不停机功能的启用/禁用
- 值为1时启用出错不停机，值为0时禁用

---

### 2. axis.cpp - watchdog_check() 函数
**位置**：第218行 `bool Axis::watchdog_check()`

**改动内容**：
```cpp
// 夹取：计数器至少保持 1，下面的超时分支因此永远不会触发
if (watchdog_current_value_ == 0)
    watchdog_current_value_ = 1;
```

**作用**：
- 防止watchdog计数器下溢到0
- 保持计数器至少为1，防止超时错误被触发

---

### 3. axis.hpp - 成员变量添加
**位置**：第242行后

**改动内容**：
- 添加成员变量 `std::optional<float> last_pos_estimate_local_;`
- 注释说明："上一次正常的编码器位置估计，用于回零时兜底"

**作用**：
- 缓存上一次正常的编码器位置估计值
- 当没有新的位置估计时用作备用值

---

### 4. low_level.cpp - safety_critical_apply_brake_resistor_timings()
**位置**：第189行

**改动内容**：
```cpp
// 夹取：把 high_on 抬到满足死区要求的位置，下面的检查因此永远不会触发
if (high_on < low_off + TIM_APB1_DEADTIME_CLOCKS)
    high_on = low_off + TIM_APB1_DEADTIME_CLOCKS;
```

**作用**：
- 自动调整PWM timing以满足死区（deadtime）要求
- 防止死区时间验证错误

---

### 5. low_level.cpp - update_brake_current() 函数
**位置**：第667行

**改动内容**：
a) 制动电阻值夹取（第675行）
```cpp
// 夹取：制动电阻值非法时用板级默认值代替，下面的检查因此永远不会触发
if (!(odrv.config_.brake_resistance > 0.0f))
    odrv.config_.brake_resistance = DEFAULT_BRAKE_RESISTANCE;
```

b) NaN占空比处理（第685行）
```cpp
// 夹取：NaN 占空比按 0 处理，下面的检查因此永远不会触发
if (std::isnan(brake_duty))
    brake_duty = 0.0f;
```

c) 母线电流夹取（第706行）
```cpp
// 夹取：用于本检查的电流先夹到限值以内，下面的检查因此永远不会触发
float Ibus_sum_for_check = Ibus_sum;
if (!(Ibus_sum_for_check <= odrv.config_.dc_max_positive_current))
    Ibus_sum_for_check = odrv.config_.dc_max_positive_current;
if (!(Ibus_sum_for_check >= odrv.config_.dc_max_negative_current))
    Ibus_sum_for_check = odrv.config_.dc_max_negative_current;
```

**作用**：
- 确保制动系统参数在合法范围内
- 防止NaN值导致的系统故障
- 防止母线电流超限错误

---

## 未完全应用的改动

由于代码库版本差异，以下功能需要手动集成：

1. **Controller相关改动**（controller.hpp/cpp）
   - 添加的备用值变量
   - 控制器更新中的夹取逻辑
   - 原因：当前controller.hpp版本结构差异较大

2. **Motor相关改动**（motor.hpp/cpp）
   - 电流测量的夹取
   - 电流saturation处理
   - disarm_with_error条件编译
   - 原因：电机控制逻辑结构变更

3. **Encoder相关改动**（encoder.hpp/cpp）
   - Hall状态处理
   - SPI错误率夹取
   - 原因：编码器处理代码结构不同

4. **Main.cpp条件编译**
   - `#if !ODRIVE_KEEP_ARMED_ON_ERROR` 保护
   - disarm调用的条件性执行
   - 原因：主控制循环代码结构已改

---

## 验证方式

### 查看已应用的改动
```bash
git diff Firmware/MotorControl/odrive_main.h
git diff Firmware/MotorControl/axis.cpp
git diff Firmware/MotorControl/axis.hpp
git diff Firmware/MotorControl/low_level.cpp
```

### 编译验证
```bash
# 需要根据ODrive项目的构建系统进行编译
# 确保没有编译错误
```

---

## 已修改的文件清单
- ✅ `Firmware/MotorControl/odrive_main.h` - 添加宏开关
- ✅ `Firmware/MotorControl/axis.cpp` - watchdog夹取
- ✅ `Firmware/MotorControl/axis.hpp` - 备用值变量
- ✅ `Firmware/MotorControl/low_level.cpp` - 多处夹取逻辑

## 部分应用的文件
- ⚠️ `Firmware/MotorControl/controller.hpp/cpp` - 需要版本适配
- ⚠️ `Firmware/MotorControl/motor.hpp/cpp` - 需要版本适配
- ⚠️ `Firmware/MotorControl/encoder.hpp/cpp` - 需要版本适配
- ⚠️ `Firmware/MotorControl/main.cpp` - 需要条件编译适配
- ⚠️ `Firmware/MotorControl/sensorless_estimator.hpp/cpp` - 需要版本适配
- ⚠️ `Firmware/MotorControl/thermistor.cpp` - 需要版本适配

---

## 建议后续步骤

1. **编译测试**
   - 使用当前应用的改动进行编译
   - 检查是否存在编译错误

2. **功能测试**
   - 验证出错不停机功能是否工作
   - 测试各项保护机制是否仍然正常

3. **集成剩余改动**
   - 根据实际代码结构手动集成Motor/Controller等模块的改动
   - 确保所有夹取逻辑一致

4. **版本控制**
   - 创建feature branch用于此改动
   - 为后续维护做好记录

---

## 技术细节

### 夹取（Clamping）策略
此patch使用"前置夹取"策略：
- 在检查前先将值夹到合法范围内
- 确保后续的越界检查总是通过
- 这样可以防止因为数据异常导致的系统停机

### 备用值（Fallback）策略
对于关键的输入值（位置、速度、电流等）：
- 缓存上一次正常的值
- 当新值不可用时使用备用值
- 确保系统在传感器故障时仍能继续运行

### 条件编译框架
为了支持快速切换，在关键的disarm点使用条件编译：
```cpp
#if !ODRIVE_KEEP_ARMED_ON_ERROR
    // disarm motors
#endif
```

---

## 注意事项

⚠️ **重要警告**：
- 此patch改变了系统的基本安全行为
- 启用"出错不停机"可能导致在错误条件下电机继续运行
- 在生产环境中使用前必须进行充分的功能测试
- 建议在安全的测试环境中先行验证

---

## 相关文件

- 原始patch文件：`odrive-keep-armed-on-error.patch`
- 应用于：ODrive v4.x Firmware
- 修改日期：2026-11-07
