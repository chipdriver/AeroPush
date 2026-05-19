# 基于当前工程的磁力计椭球标定学习笔记

这份笔记结合当前工程文件 `E:\Stm32Project\AeroPush\User\Src\mpu9250_driver.c` 来整理，重点不是先堆数学，而是先建立一个工程上真正能用的理解框架：

- 现在这套磁力计标定怎么用；
- 它相比旧方案到底强在哪里；
- 你应该看哪些日志；
- 它怎样一路参与到 `mag_yaw` 和 `fused yaw`；
- 以后如果想继续深入，哪些数学再回头学。

## 这次升级到底升级了什么

你原来的思路可以概括成：

```c
corrected_x = (raw_x - offset_x) * scale_x;
corrected_y = (raw_y - offset_y) * scale_y;
corrected_z = (raw_z - offset_z) * scale_z;
```

这类方法本质上做两件事：

- 用 `offset` 消除硬铁偏移；
- 用每个轴独立的 `scale` 把三个轴拉到相近幅值（软铁校准）。

这对“轴向仍然和坐标轴对齐”的椭球是够用的。

![image-20260518204038908](C:\Users\w1785\AppData\Roaming\Typora\typora-user-images\image-20260518204038908.png)

但真实磁场畸变常常不是这么简单。软铁干扰可能让点云：

- 被拉伸；
- 被压扁；
- 还发生旋转；
- 使 X/Y/Z 之间产生耦合。

这时点云不再只是“沿 X、Y、Z 三个方向独立拉伸”，而是一个歪着的椭球。旧方案只能做“对角线缩放”，不能把一个旋转过的椭球真正变回球。

你这次升级后，变成：

```c
corrected = M * (raw - center)
```

其中：

- `center`：椭球中心，解决硬铁偏移；
- `M`：3×3 软铁校正矩阵，解决缩放不一致、轴间耦合和旋转问题。
- `raw` ：磁力计自己的原始三轴测量数据

这就是这次升级最重要的工程意义：

> 旧方案只能“把三个轴各自拉一拉”；  
> 新方案可以“把整个歪椭球重新扳正并压回球”。

## 第一层：你现在必须会用什么

你现在先不用推导矩阵，只要先会做三件事：

1. 正确采样；
2. 正确看日志；
3. 正确验证结果。

## 第一层：如何正确做一次标定

你的初始化链路里会自动调用：

```c
MPU9250_Driver_Init()
    -> AK8963_CalibrateMag(500U, 20U)
```

当前一次标定时长约为：

```text
500 * 20ms = 10 秒
```

你已经通过实际试验验证出：想让 3D 椭球拟合成功，关键不是“乱晃十秒”，而是要让点云覆盖尽量完整的三维球面。

你最后成功那次，最关键的日志是：

```text
seed center=66.0/35.2/47.0 radius=55.6/61.8/54.1 avg=57.2
calibrate mag ok center=78.3/29.4/29.3 radius=65.6 fit=0.0049
```

为什么这次能成功？

因为三个轴的初始半径终于比较接近：

```text
55.6 / 61.8 / 54.1
```

而之前失败时常见的是：

```text
41 / 56 / 57
38 / 45 / 56
59 / 27 / 55
```

这些都说明有某一轴没有扫开。

实际操作时，可以按这个动作流程：

```text
1. 水平转一圈
2. 前后翻滚一圈
3. 左右翻滚一圈
4. 再做两组斜 45° 的连续转动，补球面空白区域
```

目标不是“动作好看”，而是让三轴范围都打开。

## 第一层：怎么看标定日志

当前代码里，标定结束后会打印几类关键日志。

一次成功日志如下：

```text
[AK8963] raw bounds x=10.4..121.5 y=-26.6..96.9 z=-7.2..101.1
[AK8963] seed center=66.0/35.2/47.0 radius=55.6/61.8/54.1 avg=57.2
[AK8963] calibrate mag ok center=78.3/29.4/29.3 radius=65.6 fit=0.0049
[AK8963] mag matrix row0=1.013,0.067,0.036
[AK8963] mag matrix row1=0.067,1.074,0.081
[AK8963] mag matrix row2=0.036,0.081,0.944
[AK8963] mag calibration done
```

你要会看这几项：

| 日志项 | 你该怎么理解 |
|---|---|
| `raw bounds` | 这次采样在三个轴上实际扫到了多大范围 |
| `seed center` | 由 min/max 粗估出来的初始中心 |
| `radius=x/y/z` | 三轴粗半径，越接近越好 |
| `avg` | 三轴粗半径平均值 |
| `center` | 最终椭球拟合得到的中心 |
| `radius` | 拟合后目标球半径 |
| `fit` | 拟合误差，越小越好 |
| `matrix row0/1/2` | 3×3 软铁矩阵的三行 |
| `mag calibration done` | 真正完成校准 |

## 第一层：`fit` 和 `radius` 怎么判断是否异常

结合这次成功经验，可以先用下面的工程标准判断。

| 项目 | 判断方式 |
|---|---|
| `radius=x/y/z` 粗半径 | 三个轴越接近越好 |
| 最终 `radius` | 应该和 `seed avg` 同量级 |
| `fit` | 越小越好，过大通常说明拟合质量差 |

例如：

```text
55.6/61.8/54.1
```

这是比较健康的三轴覆盖。

而：

```text
59.5/27.3/55.2
```

就说明 Y 轴明显没有扫开。

你的代码里已经做了基本保护：

```c
if ((fitted_radius_ut < (radius_avg * 0.5f)) ||
    (fitted_radius_ut > (radius_avg * 1.5f)))
{
    // 判定为不可信拟合
}
```

你成功那次的：

```text
fit=0.0049
```

说明模型和点云贴合得不错。

## 第一层：重新烧录后怎么做北东南西验证

建议流程：

1. 重新烧录；
2. 上电后完成磁力计校准，确认出现：
   ```text
   calibrate mag ok
   mag calibration done
   ```
3. 板子尽量放平；
4. 分别让板子朝：
   - 北
   - 东
   - 南
   - 西
5. 每个方向静止几秒，看稳定后的：
   - `mag_yaw`
   - `fused yaw`

理想上希望看到：

| 方向 | 期望 |
|---|---:|
| 北 | 约 `0°` |
| 东 | 约 `90°` |
| 南 | 约 `180°` 或 `-180°` |
| 西 | 约 `-90°` |

你这次实测已经说明系统基本可用：

```text
北：mag_yaw=2.7
西：mag_yaw=-90.5
南：mag_yaw=-172.6
东：mag_yaw=100.9
```

更重要的是：

```text
fused yaw 和 mag_yaw 基本贴合
```

这说明校准后的磁力计已经能为姿态融合提供有效航向参考。

## 第二层：完整数据流图

当前工程中的磁力计数据流可以写成：

```text
AK8963 raw data
    AK8963 的原始磁力计数据，通常是寄存器里的 raw_x / raw_y / raw_z

    ->

AK8963_Read_Axis()
    从 AK8963 寄存器读取三轴原始数据 raw_x / raw_y / raw_z

    ->

ak8963_convert_raw_to_ut()
    将 raw 原始值转换成 uT 单位的磁场物理量

    ->

center 校正：mag_ut - center
    将整个点云的中心拉回原点，也就是硬铁校准

    ->

3×3 矩阵校正：M * (mag_ut - center)
    在硬铁校准基础上进行软铁校正，修正椭球拉伸、压扁、旋转和轴间耦合

    ->

corrected mag
    得到已经修正过的磁力计数据 corrected_x / corrected_y / corrected_z

    ->

MPU9250_ComputeEuler_FromAccMag()
    用加速度计和校正后的磁力计直接计算一组参考欧拉角

    ->

mag_yaw
    根据校正后的磁力计数据，算出一个磁力计参考航向角

    ->

MPU9250_MahonyUpdate()
    Mahony 融合陀螺仪、加速度计、磁力计，更新最终姿态

    ->

fused yaw
    Mahony 融合后的最终航向角
```

结合任务层调用链，再展开就是：

```text
ImuTask
    ->
ImuService_ReadPhys()
    ->
AK8963_Read_Mag_UT()
    ->
AK8963_Calibrate()
    ->
ak8963_convert_raw_to_ut()
    ->
ak8963_apply_mag_calibration()
    ->
mag_physical
    ->
MPU9250_ComputeEuler_FromAccMag()
    ->
mag_yaw

同时：

mag_physical
    ->
MPU9250_MahonyUpdate()
    ->
fused yaw
```

## 第二层：原始磁力计数据是怎么一路流到 yaw 的

当前读取链路的核心代码如下：

```c
int AK8963_Read_Axis(AK8963_raw_Data *raw)
{
    uint8_t frame[7]; // 一次性读出 HXL..ST2 共 7 字节

    if (raw == 0)
    {
        return -2; // 空指针保护
    }

    if (AK8963_CheckDataReady() == 0)
    {
        return -1; // 数据未就绪
    }

    if (I2C_ReadRegs(AK8963_I2C_ADDR7, AK8963_REG_HXL,
                     frame, (uint16_t)sizeof(frame)) != 0)
    {
        return -2; // I2C 读取失败
    }

    raw->mag_x = (int16_t)(((uint16_t)frame[1] << 8U) | frame[0]);
    raw->mag_y = (int16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    raw->mag_z = (int16_t)(((uint16_t)frame[5] << 8U) | frame[4]);

    if ((frame[6] & 0x08U) != 0U)
    {
        return -3; // 磁力计溢出
    }

    return 0;
}
```

逐行理解：

| 代码 | 工程意义 |
|---|---|
| `frame[7]` | 一次突发读取 HXL 到 ST2，确保同一帧数据完整 |
| `AK8963_CheckDataReady()` | 防止读取旧数据 |
| `I2C_ReadRegs(...)` | 从 AK8963 中取原始寄存器值 |
| `raw->mag_x/y/z` | 把低字节、高字节组装为三轴原始值 |
| `frame[6] & 0x08U` | 检查溢出位，避免异常数据进入后续链路 |

接下来，原始值先转成物理单位，再做校准：

```c
void AK8963_Calibrate(const AK8963_raw_Data *raw,
                      AK8963_Physical_Data *physical)
{
    ak8963_convert_raw_to_ut(raw, physical); // 原始计数 -> uT
    ak8963_apply_mag_calibration(physical);  // 应用 center + 3×3 矩阵
}
```

在线运行时真正被调用的是：	

```c
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out)
{
    AK8963_raw_Data raw;
    int ret;

    if (mag_out == 0)
    {
        return -2;
    }

    ret = AK8963_Read_Axis(&raw); // 先读原始数据
    if (ret != 0)
    {
        return ret;
    }

    AK8963_Calibrate(&raw, mag_out); // 再转成校正后的 uT
    return 0;
}
```

也就是说，后面的 yaw 和 Mahony 拿到的已经不是原始磁场，而是校正后的磁场。

## 第二层：`center` 到底表示什么

在当前代码里，最终拟合中心保存在：

```c
g_mag_offset_ut[0]
g_mag_offset_ut[1]
g_mag_offset_ut[2]
```

虽然变量名还叫 `offset`，但在新的算法语义里，它已经等价于：

```text
center = 椭球中心
```

它代表的是：

> 受硬铁干扰后，整个磁场点云相对于坐标原点被平移了多少。

如果理想磁场点云应该围绕原点成球，但实际采样却围绕某个偏移位置成椭球，那么这个偏移位置就是 `center`。

这次成功拟合得到：

```text
center=78.3/29.4/29.3
```

后续必须先做：

```c
raw - center
```

才能把点云重新拉回原点附近。

## 第二层：原来的 `(mag - offset) * scale` 能解决什么，不能解决什么

旧方案可以解决：

- 硬铁偏移；
- 每个轴增益不同；
- 点云是“轴对齐椭球”的情况。

也就是：

```text
原点被平移了
X、Y、Z 各自被不同程度拉伸了
但椭球主轴仍然和传感器坐标轴平行
```

旧方案解决不了：

- 软铁导致的轴间耦合；
- 椭球主轴发生旋转；
- 水平面投影不再只是简单的 X/Y 缩放；
- 某些角度被压缩，某些角度被拉伸。

这就是为什么你之前会出现四方向角度间隔类似：

```text
58° / 59° / 118° / 123°
```

理想情况下，北东南西相邻方向应该接近：

```text
90° / 90° / 90° / 90°
```

如果被拉成椭圆，那么：

- 在椭圆长轴附近，角度变化会显得“慢”；
- 在短轴附近，角度变化会显得“快”。

于是你明明把板子物理转了 90°，但 `mag_yaw` 可能只走了 58°；再转另一个 90°，它可能一下走了 123°。

这说明：

> 磁场在水平面的投影已经不是圆，而是椭圆。  
> 单纯每轴独立 scale 未必足够把它彻底修正。

## 第二层：`M` 这个 3×3 软铁矩阵表示什么

你的矩阵保存在：

```c
g_mag_correction[3][3]
```

它是本次升级的核心。

从工程角度看，`M` 同时做了三件事：

1. 调整三个轴的比例；
2. 修正轴之间的串扰；
3. 把“歪着的椭球”旋回正确方向。

如果只是每轴独立 scale，矩阵会长这样：

```text
[ sx   0   0 ]
[  0  sy   0 ]
[  0   0  sz ]
```

这叫对角矩阵。

而你现在成功拟合出的矩阵是：

```text
[1.013  0.067  0.036]
[0.067  1.074  0.081]
[0.036  0.081  0.944]
```

它的非对角元素不是 0：

```text
0.067、0.036、0.081
```

这正说明：

- X 校正里掺入了一点 Y/Z；
- Y 校正里也掺入了一点 X/Z；
- Z 校正里也掺入了一点 X/Y。

这就是“轴间耦合”的工程体现。

## 第二层：`corrected = M * (raw - center)` 在代码里怎么实现

真正应用校准的是这个函数：

```c
static void ak8963_apply_mag_calibration(AK8963_Physical_Data *mag)
{
    float centered_x;
    float centered_y;
    float centered_z;
    float corrected_x;
    float corrected_y;
    float corrected_z;

    if (mag == 0)
    {
        return;
    }

    centered_x = mag->mag_x_ut - g_mag_offset_ut[0];
    centered_y = mag->mag_y_ut - g_mag_offset_ut[1];
    centered_z = mag->mag_z_ut - g_mag_offset_ut[2];

    corrected_x = g_mag_correction[0][0] * centered_x +
                  g_mag_correction[0][1] * centered_y +
                  g_mag_correction[0][2] * centered_z;

    corrected_y = g_mag_correction[1][0] * centered_x +
                  g_mag_correction[1][1] * centered_y +
                  g_mag_correction[1][2] * centered_z;

    corrected_z = g_mag_correction[2][0] * centered_x +
                  g_mag_correction[2][1] * centered_y +
                  g_mag_correction[2][2] * centered_z;

    mag->mag_x_ut = corrected_x;
    mag->mag_y_ut = corrected_y;
    mag->mag_z_ut = corrected_z;
}
```

逐行理解：

| 代码 | 说明 |
|---|---|
| `centered_x = mag_x - center_x` | 先做硬铁补偿 |
| `corrected_x = row0 · centered_vector` | 用矩阵第 0 行算校正后的 X |
| `corrected_y = row1 · centered_vector` | 用矩阵第 1 行算校正后的 Y |
| `corrected_z = row2 · centered_vector` | 用矩阵第 2 行算校正后的 Z |
| 最后三行赋值 | 用校正结果覆盖原始磁场值，供后续 yaw / Mahony 使用 |

如果用数学写，就是：

```text
[ corrected_x ]   [ m00 m01 m02 ] [ raw_x - center_x ]
[ corrected_y ] = [ m10 m11 m12 ] [ raw_y - center_y ]
[ corrected_z ]   [ m20 m21 m22 ] [ raw_z - center_z ]
```

## 第二层：`matrix row0 / row1 / row2` 日志怎么读

成功时打印：

```text
matrix row0=1.013,0.067,0.036
matrix row1=0.067,1.074,0.081
matrix row2=0.036,0.081,0.944
```

把它还原成矩阵就是：

```text
M =
[1.013  0.067  0.036]
[0.067  1.074  0.081]
[0.036  0.081  0.944]
```

你可以这样看：

- 对角线：
  - `1.013`
  - `1.074`
  - `0.944`
  
  表示三个主方向的缩放修正幅度。

- 非对角线：
  - `0.067`
  - `0.036`
  - `0.081`
  
  表示三个轴之间存在一定耦合，需要互相混合后才能把椭球校正成球。

如果某次日志出现：

```text
row0=1,0,0
row1=0,1,0
row2=0,0,1
```

那说明几乎没有软铁修正，只是在做平移。  
如果非对角项非常大，也要警惕：

- 采样不充分；
- 拟合异常；
- 周围磁干扰强。

## 第二层：标定阶段到底做了什么

主函数是：

```c
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms)
```

它主要做了这些事：

```c
for (i = 0U; i < samples; i++)
{
    read_ret = AK8963_Read_Axis(&raw);

    if (read_ret == 0)
    {
        ak8963_convert_raw_to_ut(&raw, &mag);

        g_mag_cal_samples[valid_count] = mag;

        if (mag.mag_x_ut < min_x) min_x = mag.mag_x_ut;
        if (mag.mag_x_ut > max_x) max_x = mag.mag_x_ut;
        if (mag.mag_y_ut < min_y) min_y = mag.mag_y_ut;
        if (mag.mag_y_ut > max_y) max_y = mag.mag_y_ut;
        if (mag.mag_z_ut < min_z) min_z = mag.mag_z_ut;
        if (mag.mag_z_ut > max_z) max_z = mag.mag_z_ut;

        valid_count++;
    }
}
```

逐步理解：

| 阶段 | 工程意义 |
|---|---|
| 连续采样 | 收集一批三维点 |
| 保存到 `g_mag_cal_samples[]` | 给后面的拟合器使用 |
| 同时统计 min/max | 给拟合器一个合理初值，也用于基本保护 |
| 打印 raw bounds / radius | 帮你判断采样是否覆盖完整 |

之后代码先用 min/max 生成一个“初始猜测”：

```c
fit_params[0] = (min_x + max_x) * 0.5f;
fit_params[1] = (min_y + max_y) * 0.5f;
fit_params[2] = (min_z + max_z) * 0.5f;

fit_params[3] = 1.0f / (radius_x * radius_x);
fit_params[4] = 0.0f;
fit_params[5] = 0.0f;
fit_params[6] = 1.0f / (radius_y * radius_y);
fit_params[7] = 0.0f;
fit_params[8] = 1.0f / (radius_z * radius_z);
```

意思是：

- 先假设椭球中心就在 min/max 中点；
- 先假设椭球还没有旋转；
- 再交给 `ak8963_fit_ellipsoid()` 去迭代优化。

然后：

```c
ak8963_fit_ellipsoid(...)
ak8963_build_softiron_matrix(...)
```

最后得到：

- `center`
- `M`
- `fit_error`
- `fitted_radius_ut`

## 第二层：为什么四方向角度间隔会暴露椭球问题

你之前观察到过类似：

```text
58° / 59° / 118° / 123°
```

理想情况下，磁场水平投影是一条圆：

```text
北 -> 东 -> 南 -> 西
```

每一段都应该接近：

```text
90°
```

但如果原始磁场投影被拉成椭圆：

- 某些方向的角变化会被“压缩”；
- 某些方向的角变化会被“拉伸”。

于是你明明把板子物理转了 90°，但 `mag_yaw` 可能只走了 58°；再转另一个 90°，它可能一下走了 123°。

这说明：

> 你的 yaw 映射不是线性的，磁场点云在水平面上已经变形。

这正是为什么你需要从：

```text
offset + 每轴独立 scale
```

升级到：

```text
center + 3×3 矩阵
```

## 第二层：校正后的磁场怎样参与 `mag_yaw`

直接磁航向计算在：

```c
MPU9250_ComputeEuler_FromAccMag()
```

核心过程是：

```c
mx =  mag->mag_x_ut;
my = -mag->mag_y_ut;
mz = -mag->mag_z_ut;
```

这里的 `mag` 已经是校正后的磁场。

接下来先通过加速度计算 `roll`、`pitch`，再做倾斜补偿：

```c
mx2 = mx * cosPitch + mz * sinPitch;
my2 = mx * sinRoll * sinPitch +
      my * cosRoll -
      mz * sinRoll * cosPitch;

g_euler_acc_mag.yaw = atan2f(-my2, mx2);
```

逐步理解：

| 步骤 | 意义 |
|---|---|
| 用加速度求 roll/pitch | 知道当前板子是不是倾斜 |
| 把磁场投影回水平面 | 不让倾斜影响航向 |
| `atan2f(-my2, mx2)` | 求出磁航向角 |
| `MPU9250_GetEulerDeg()` | 再转成项目里的导航角定义 |

所以：

```text
mag_yaw
```

不是单纯看 XY 原始磁场，而是看经过三维校正和倾斜补偿后的磁场方向。

## 第二层：校正后的磁场怎样参与 Mahony 融合

在 `ImuTask` 中：

```c
if (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK)
{
    MPU9250_ComputeEuler_FromAccMag(&phys, &mag_physical);
    MPU9250_GetEulerDeg(&mag_roll_deg, &mag_pitch_deg, &mag_yaw_deg);
    MPU9250_MahonyUpdate(&phys, &mag_physical, imu_dt);
}
```

这说明：

- `mag_yaw` 是为了观察磁力计本身算出来的直接航向；
- `MahonyUpdate()` 则把同一份校正后的磁场用于融合。

在 `MPU9250_MahonyUpdate()` 中：

```c
mx =  mag->mag_x_ut;
my = -mag->mag_y_ut;
mz = -mag->mag_z_ut;
```

然后它会：

1. 归一化磁场向量；
2. 估计当前姿态下理论磁场方向；
3. 用“测得磁场”和“理论磁场”的误差修正姿态；
4. 从而抑制 yaw 长期漂移。

这就是为什么成功标定后会看到：

```text
fused yaw 和 mag_yaw 基本贴合
```

这说明磁力计已经能作为融合里的长期航向参考源。

## 第三层：以后再深入的数学内容

现在你不必立刻死磕这些推导，但以后可以继续学：

| 数学主题 | 以后学它的意义 |
|---|---|
| 椭球隐式方程 | 理解 9 个拟合参数到底在表示什么 |
| 雅可比矩阵 | 理解 `ak8963_build_fit_system()` 怎么构造线性化系统 |
| Levenberg-Marquardt / 阻尼最小二乘 | 理解 `damping` 为什么会增减 |
| 对称矩阵特征值分解 | 理解 `ak8963_build_softiron_matrix()` 为什么能从椭球得到 `M` |
| 正定矩阵 | 理解为什么必须检查椭球矩阵的特征值 |
| 数值稳定性 | 理解为什么采样覆盖不全时拟合会跑飞 |

当前阶段最该先建立的是：

> 先知道“这段代码在工程上要解决什么问题”，再回头补数学推导。

## 当前这套代码里的关键保护逻辑

这次升级并不是“只拟合，不保护”。代码里已经有不少很实用的防线。

```c
if (valid_count < AK8963_MAG_FIT_MIN_VALID_SAMPLES)
{
    return;
}
```

防止有效点太少。

```c
if ((radius_x < AK8963_MAG_RADIUS_MIN_UT) ||
    (radius_y < AK8963_MAG_RADIUS_MIN_UT) ||
    (radius_z < AK8963_MAG_RADIUS_MIN_UT))
{
    return;
}
```

防止某一轴根本没展开。

```c
if (ak8963_fit_ellipsoid(...) == 0U)
{
    return;
}
```

防止拟合器失败后继续使用垃圾结果。

```c
if (ak8963_build_softiron_matrix(...) == 0U)
{
    return;
}
```

防止无效椭球生成校正矩阵。

```c
if ((fit_params[0] < min_x) || (fit_params[0] > max_x) ||
    (fit_params[1] < min_y) || (fit_params[1] > max_y) ||
    (fit_params[2] < min_z) || (fit_params[2] > max_z) ||
    (fitted_radius_ut < (radius_avg * 0.5f)) ||
    (fitted_radius_ut > (radius_avg * 1.5f)))
{
    return;
}
```

防止“数学上能拟合，物理上不可信”的结果进入系统。

## 在 STM32 工程里的风险点

这套 3D 椭球拟合比旧方案强，但在嵌入式里也更重。

| 风险点 | 具体体现 |
|---|---|
| RAM 占用 | `g_mag_cal_samples[500]`、9×9 矩阵、多个向量都会占 RAM |
| 采样数量 | 500 点对稳定拟合有帮助，但时间更长 |
| 栈空间 | 局部数组和局部变量较多，后续扩展时要关注任务栈 |
| 数值条件 | 采样覆盖不均时，矩阵容易病态，拟合会失败或跑飞 |
| 磁干扰 | 靠近铁件、电机、电流线束会让“真实磁场”本身发生变化 |
| 拟合失败保护 | 如果没有保护，错误矩阵会直接污染 yaw |
| 运行时有效性 | 即使标定成功，环境一变，磁场也可能被干扰 |
| CPU 开销 | 标定阶段会做迭代、求解、特征分解，不适合高频实时路径 |
| 数据质量 | 长时间停在某个姿态，会让点云分布不均匀 |
| 参数持久化 | 若以后存 Flash，要考虑版本、校验、失效机制 |

你这次实际踩到最多的是：

```text
采样覆盖不均 -> 拟合失败 / 拟合结果不可信
```

最后成功那轮，正是因为三轴覆盖终于均衡。

## 你这次升级带来的工程收益

这次从旧方案升级后，你得到的不只是“更高级的数学”，而是几个很实际的好处：

1. 四象限航向间隔会更接近真实；
2. 被旋转过的软铁畸变也能修；
3. `mag_yaw` 的线性会更好；
4. Mahony 拿到的磁场方向更可靠；
5. 长期 yaw 漂移更容易被正确拉回；
6. 对复杂机械安装环境更有适应性。

一句话总结：

> 旧方案让磁力计“能用”；  
> 新方案让磁力计更有资格成为姿态融合里的可靠航向参考。

## 你现在应该形成的最小知识闭环

你现在至少要能自己说清这几句话：

1. `AK8963_Read_Axis()` 读的是原始三轴寄存器数据；
2. `ak8963_convert_raw_to_ut()` 把它变成 uT；
3. `AK8963_CalibrateMag()` 通过一批姿态采样，拟合出 `center` 和 `M`；
4. `ak8963_apply_mag_calibration()` 执行：
   ```text
   corrected = M * (raw - center)
   ```
5. `MPU9250_ComputeEuler_FromAccMag()` 用校正后的磁场算 `mag_yaw`；
6. `MPU9250_MahonyUpdate()` 用同一份校正后的磁场约束融合 yaw；
7. 如果四方向不是接近 90° 间隔，说明磁场投影仍可能不是圆；
8. 看标定日志时，先看：
   - `radius x/y/z`
   - `fit`
   - `matrix`
   - 是否真正 `mag calibration done`
