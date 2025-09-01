# 3D MPCC vs MPC 轨迹跟踪仿真

这个项目实现了3D轨迹跟踪中MPCC（Model Predictive Contouring Control）和MPC（Model Predictive Control）的对比仿真。

## 项目结构

```
MPCC_3D/
├── main_simulation.m          # 主仿真文件
├── define_3d_trajectory.m     # 3D轨迹定义函数
├── geom3_at.m                 # 3D几何工具函数
├── step_dyn.m                 # 动力学更新函数
├── clamp_u.m                  # 控制输入限制函数
├── clamp_pitch.m              # 俯仰角限制函数
├── update_arc_length.m        # 弧长更新函数
├── sample_ref_path3D_time.m   # 时间相关参考路径采样（MPC用）
├── solve_tracking_mpc_3d.m    # MPC求解函数
├── linearize_batch3D.m        # 批量线性化函数
├── condensed_dynamics.m       # 凝聚动力学函数
├── solve_mpcc_3d.m            # MPCC求解函数
├── initialize_visualization.m # 可视化初始化函数
├── update_visualization.m     # 可视化更新函数
├── calculate_final_stats.m    # 统计结果计算函数
└── README.md                  # 项目说明文档
```

## 主要特性

### 1. 轨迹类型
- **直线** (`line`): 从(0,0,0)到(L,0,0)的3D直线
- **螺旋线** (`helix`): 3D螺旋轨迹
- **利萨如图形** (`lissajous`): 3D利萨如图形轨迹

### 2. 控制器对比
- **MPC**: 传统模型预测控制，基于时间跟踪
- **MPCC**: 模型预测轮廓控制，基于弧长优化

### 3. 关键改进
- **MPC修改**: 从弧长相关改为时间相关
- **模块化设计**: 每个文件只包含一个函数，符合MATLAB规范
- **清晰接口**: 每个函数都有详细的输入输出说明

## 使用方法

### 1. 运行仿真
```matlab
% 直接运行主文件
main_simulation
```

### 2. 修改参数
在 `main_simulation.m` 中修改以下参数：
```matlab
traj_type = 'helix';    % 轨迹类型
Tf        = 25.0;       % 仿真时间
dt        = 0.05;       % 时间步长
N         = 20;         % 预测步数
```

### 3. 调整权重
```matlab
% MPCC权重
Wc   = 50;   % 轮廓误差权重
Wl   = 1.0;  % 滞后误差权重
Wv   = 0.5;  % 速度跟踪权重

% MPC权重
Qmpc = diag([15,15,15, 2,2, 0.5]); % 状态误差权重
Rmpc = diag([1e-3,1e-3,1e-3]);     % 控制输入权重
```

## 技术细节

### 状态向量
- `x = [px, py, pz, yaw, pitch, v]`
- `px, py, pz`: 3D位置坐标
- `yaw, pitch`: 航向角和俯仰角
- `v`: 标量速度

### 控制输入
- `u = [a, w_yaw, w_pitch]`
- `a`: 加速度
- `w_yaw`: 偏航角速度
- `w_pitch`: 俯仰角速度

### MPCC vs MPC 区别

| 特性 | MPC | MPCC |
|------|-----|------|
| 参考生成 | 基于时间 | 基于弧长 |
| 误差定义 | 位置/姿态误差 | 轮廓/滞后误差 |
| 优化变量 | 控制输入 | 控制输入+弧长增量 |
| 适用场景 | 精确跟踪 | 路径跟随 |

## 依赖要求

- MATLAB R2018b或更高版本
- Optimization Toolbox（用于quadprog）
- Statistics and Machine Learning Toolbox（用于table）

## 文件说明

### `main_simulation.m`
主仿真文件，包含：
- 参数设置
- 轨迹定义
- 主循环
- 结果统计

### `define_3d_trajectory.m`
定义3D轨迹的函数，支持：
- 直线轨迹
- 螺旋轨迹
- 利萨如图形轨迹

### `sample_ref_path3D_time.m`
基于时间采样参考路径（MPC用）：
- 根据时间生成参考轨迹
- 不再依赖弧长进度

### `solve_tracking_mpc_3d.m`
MPC求解函数：
- 求解3D跟踪MPC问题
- 使用线性化和凝聚动力学

### `solve_mpcc_3d.m`
MPCC求解函数：
- 求解MPCC问题
- 包含轮廓误差和滞后误差优化

### `step_dyn.m`
3D点质量模型的动力学更新函数

### `clamp_u.m`
限制控制输入在允许范围内的函数

### `clamp_pitch.m`
限制俯仰角在合理范围内的函数

### `update_arc_length.m`
根据当前位置更新弧长进度的函数

### `linearize_batch3D.m`
批量线性化3D动力学的函数

### `condensed_dynamics.m`
构建凝聚动力学矩阵的函数

### `initialize_visualization.m`
初始化可视化界面的函数

### `update_visualization.m`
更新可视化显示的函数

### `calculate_final_stats.m`
计算最终统计结果的函数

## 注意事项

1. **MPC修改**: MPC现在基于时间生成参考轨迹，而不是弧长
2. **弧长更新**: MPCC仍然使用优化的弧长增量
3. **性能对比**: 可以通过修改权重参数来调整两种控制器的性能
4. **可视化**: 实时显示3D轨迹、控制输入和误差
5. **文件规范**: 每个文件只包含一个函数，符合MATLAB标准

## 扩展建议

1. 添加更多轨迹类型
2. 实现自适应权重调整
3. 添加噪声和扰动测试
4. 实现多智能体对比
5. 添加性能指标分析
