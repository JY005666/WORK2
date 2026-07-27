# 交接文档

更新时间：2026-07-26

## 1. 本轮主要做了什么

这轮主要围绕两条线展开：

1. 底盘距离速度规划 `Distance_Speed_Plan()`
2. 上层状态机 `UpperState`

核心目标是排查并缓解这类问题：

- 由远到近时，底盘有时不减速，直接撞挡板
- 某些阶段中，距离规划内部看起来“还离目标很远”，于是持续给大速度
- 状态机阶段切换、云台避障转向、送豆逻辑不好读，想整体梳理

---

## 2. 已完成的代码改动

### 2.1 距离样本筛选逻辑已简化

文件：

- `Usercode/DJI/Caculate.c`
- `Usercode/DJI/Caculate.h`

之前存在一套“斜率限制 + candidate 暂存 + 保留旧值”的逻辑，已删除。

现在 `reliable_distance` 的逻辑是：

1. 原始值必须是可用浮点
2. 原始值必须落在有效区间内
3. 如果值有效，则直接接纳
4. 如果值无效或越界，则低速时短时间保留上一次有效值

也就是说，现在已经没有：

- candidate 候选观察
- 斜率限制接纳
- 连续候选累积

### 2.2 加了“方案 A”版本的无效测距处理

文件：

- `Usercode/DJI/Caculate.h`
- `Usercode/DJI/Caculate.c`

新增参数：

- `DIST_SERVO_INVALID_HOLD_RPM`

当前逻辑：

1. 如果测距无效/越界
2. 且当前底盘电机转速绝对值大于 `DIST_SERVO_INVALID_HOLD_RPM`
3. 则 `DistanceServo_GetReliableSample()` 直接返回失败
4. 不再像以前那样高速时继续抱着旧距离值

低速时仍保留：

- `DIST_SERVO_INVALID_HOLD_MS` 时间内沿用旧值

### 2.3 调试打印补强

文件：

- `Usercode/Upper/Upper_state/UpperState.c`

当前 `DebugPrint()` 会打印：

- `lidar_live`：`DebugPrint()` 当场直接读取的 `lidar.distance_aver`
- `lidar`：距离规划这一拍读到的 `raw_distance` 快照
- `reliable`
- `filtered`
- `control`
- `speed_ref`
- `motor_rpm`
- `near`
- `stall`
- `mismatch`

说明：

- `stall` / `mismatch` 现在只是调试字段
- 对应保护逻辑已经回退掉
- 当前会固定打印 `0`

### 2.4 浮点绝对值错误已修正

文件：

- `Usercode/Upper/Upper_state/StateHelpers/UpperStateHelpers.c`

已把以下地方的浮点 `abs` 改成 `fabsf`：

1. `IsDistanceAndChassisReady()`
2. `LiftAndRotateToPlacement()`

注意：

- `HandleStage30_FirstBeanPlacement()` 里仍有两处 `abs(...)`
- 当前磁盘代码里还是整数 `abs`
- 这两处还没改

位置大概在：

- `HandleStage30_FirstBeanPlacement()` 的到位判断里

---

## 3. 当前距离速度规划的结构理解

相关文件：

- `Usercode/DJI/Caculate.c`
- `Usercode/DJI/Caculate.h`

### 3.1 主链路

当前距离规划的主链路是：

`lidar.distance_aver -> reliable_distance -> filtered_distance -> control_distance -> error -> stop_error -> desired_speed_ref -> last_speed_ref`

### 3.2 各变量含义

#### `reliable_distance`

含义：

- 带有效性筛选的测距值

当前规则：

1. 值正常且在有效区间：直接接纳
2. 值无效/越界：
   - 低速：短时保留旧值
   - 高速：直接失败

#### `filtered_distance`

含义：

- 对 `reliable_distance` 再做一次低通滤波后的值

当前参数：

- `DIST_FILTER_ALPHA = 0.18f`

#### `control_distance`

含义：

- 真正参与误差计算的距离

当前规则：

1. 远距离：`control_distance = filtered_distance`
2. 近距离：`control_distance = reliable_distance`

近目标切换参数：

- `DIST_SERVO_NEAR_SWITCH_IN_MM = 750`
- `DIST_SERVO_NEAR_SWITCH_OUT_MM = 850`

#### `error`

当前公式：

`error = target_distance - control_distance`

#### `stop_error`

当前公式：

`stop_error = abs_error - effective_tol`

含义：

- 扣掉允许误差带之后，真正参与刹车规划的剩余误差

#### `desired_speed_ref`

远离目标时：

1. 先算 `speed_by_p`
2. 再算 `speed_by_brake`
3. 取两者较小值
4. 再做上限、接近目标限速、最小起转速度处理

最后：

`desired_speed_ref = Sign(error) * speed_mag`

#### `last_speed_ref`

含义：

- 经过 `DistanceServo_SlewLimit()` 后的实际速度参考

这是最终发给速度环的参考值。

---

## 4. 当前距离规划仍然存在的主要隐患

### 4.1 合法但卡住的测距值，仍会被持续信任

这是当前最值得盯的隐患。

目前 `reliable_distance` 只处理：

- 无效值
- 越界值

但如果测距值：

- 合法
- 也在有效区间
- 只是长时间不变化

当前规划仍会继续把它当真。

这类现象之前出现过：

- `lidar/reliable/filtered/control` 长时间卡在某个合法值附近
- `speed_ref` 继续给很大

### 4.2 测距失败后只是“软停车”

现在高速无效测距时，`DistanceServo_GetReliableSample()` 会返回失败。

但失败后的处理是：

1. `desired_speed_ref = 0`
2. 再通过 `DistanceServo_SlewLimit()` 慢慢减速

所以它不是硬刹停，只是软停车。

### 4.3 远距离阶段仍可能有滞后

尽管滤波强度已经比以前轻了很多，但只要还没进近目标区：

- `control_distance` 还是 `filtered_distance`

所以远距离阶段仍然会有一定滞后。

---

## 5. 状态机已梳理出的主流程

相关文件：

- `Usercode/Upper/Upper_state/UpperState.c`
- `Usercode/Upper/Upper_state/StateHelpers/UpperStateHelpers.c`

### 5.1 主流程概览

整套流程可以粗记为：

1. `stage 0 -> 10`
   - 抓第一个豆子（固定先抓中间豆）

2. `stage 20 -> 30`
   - 送第一个豆子到视觉指定箱子

3. `stage 31 -> 40 -> 50`
   - 判断第二个抓左还是抓右，并抓第二个豆子

4. `stage 60 -> 61 -> 70`
   - 送第二个豆子

5. `stage 900 -> 910`
   - 抓第三个豆子

6. `stage 911 -> 920 -> 930`
   - 送第三个豆子

### 5.2 第一个豆子的特点

第一个豆子的放置逻辑是专门单写的一套：

- `HandleStage30_FirstBeanPlacement()`

特点：

1. 先用安全过渡云台角避障
2. 超过 `SAFE_DIST_FOR_BOX_FINAL_TURN_MM` 后锁定最终转向
3. 再转到最终箱位角
4. 最后半开爪放豆

### 5.3 第二、第三个豆子的特点

第二、第三个豆子的送箱逻辑共用：

- `HandleBeanDelivery()`

它负责：

1. 根据目标箱设置底盘目标距离和上爪角度
2. 根据左右跨侧情况决定是否先走避障角
3. 到位后跳转到放豆阶段

### 5.4 当前状态机里值得注意的点

#### `stage 0`

是一个两段式逻辑：

1. `lidar > 1000`
   - `target_distance = bean_middle.distance`
   - `degree_chassis = 50`

2. `lidar < 1000`
   - 切上爪角
   - `degree_chassis = bean_middle.chassis`

所以日志里出现过：

- `657 / 50`
- `657 / -3`

这是状态机设计本身导致的，不是距离规划自己乱跳。

#### `stage 30`

第一个豆子的送箱过程中，存在一次明显的“安全角 -> 最终角”切换。

当时也观察到过：

- `target_chassis` 在运行中从安全角切到最终箱位角

---

## 6. 当前调试时该怎么看日志

现在看 `DebugPrint()` 时，重点看这些字段：

1. `lidar_live`
   - 当前时刻直接读到的 `lidar.distance_aver`

2. `lidar`
   - 距离规划那一拍拿到的 `raw_distance`

3. `reliable`
   - 筛选后的距离

4. `filtered`
   - 低通后的距离

5. `control`
   - 真正用于算误差的距离

6. `speed_ref`
   - 经过斜坡后的速度参考

7. `motor_rpm`
   - 当前电机反馈转速

8. `near`
   - 当前是否进入近目标模式

### 一个常用判断方法

如果后面再出现“高速冲、不减速”的现象，优先看：

1. `lidar_live` 是否还在变
2. `lidar` 是否跟着变
3. `reliable` 是否跟着变
4. `control` 是否还卡在远处

---

## 7. 建议下个对话优先接着看的点

建议下个对话优先继续看这几件事：

1. `HandleStage30_FirstBeanPlacement()` 里两处 `abs(...)` 是否改成 `fabsf()`
2. 是否需要处理“合法但冻结的测距值”
3. 如果高速无效测距仍觉得不安全，是否把“软停车”升级为更强的停车策略
4. 如果继续分析状态机，优先看：
   - `stage 0`
   - `stage 30`
   - `stage 61 / 920`

因为这几个阶段最容易和底盘速度规划、云台角切换、测距读数一起耦合出问题。

---

## 8. 当前磁盘代码的几个关键现状

1. `DIST_FILTER_ALPHA` 当前是 `0.18f`
2. `DIST_SERVO_EFFECTIVE_MAX_MM` 当前是 `3000.0f`
3. `DIST_SERVO_INVALID_HOLD_RPM` 当前是 `800.0f`
4. `stall / mismatch` 现在只是调试字段，不参与控制
5. `stage 0` 仍是两段式抓中间豆逻辑

