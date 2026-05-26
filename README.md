# 羊体多视角点云配准与表面重建

基于 PCL 的离线 C++ 流水线：多帧 RGB 点云 → 配准对齐 → 融合去噪 → 贪婪投影三角网格。

## 方法

### 流水线总览

```
多帧 PCD ──→ 单帧预处理 ──→ 链式 ICP ──→ ELCH 闭环 ──→ LOO 细化
                                                            │
  输出 ←── GP3 网格 ←── 融合后处理 ←── 共识裁剪 + Yaw 多初值 ←─┘
```

### 1. 单帧预处理

每帧独立执行四步清洗：


| 步骤  | 算法                                | 作用           |
| --- | --------------------------------- | ------------ |
| 去噪  | Statistical Outlier Removal (SOR) | 删除稀疏离群点      |
| 平滑  | Moving Least Squares (MLS)        | 消除传感器抖动噪声    |
| 降采样 | Voxel Grid                        | 统一点密度，加速后续   |
| 去地面 | RANSAC 平面分割                       | 分离地面，仅保留羊体主体 |


### 2. 粗到精链式配准

相邻帧执行双阶段 ICP：先以大对应距离（0.12m）粗配，再以小对应距离（0.04m）精配。变换矩阵沿帧序链式累积，将全部帧统一到第 0 帧坐标系。

### 3. ELCH 闭环优化

将首尾帧视为闭合回路，利用 `pcl::registration::ELCH` 分配闭环残差到各帧，抑制链式 ICP 的累积漂移。

### 4. Leave-One-Out (LOO) 细化

每帧对"除自身外其余帧的体素融合靶"做 ICP，纠正单帧在链式/闭环后的残余偏移。默认执行 2 轮。

### 5. 共识裁剪 + 最差帧 Yaw 多初值

- **共识裁剪**：计算各帧点到共识表面的距离，删除超过阈值（38mm）的点——即错位层、悬空半截腿等伪影。
- **Yaw 多初值**：自动识别与共识偏差最大的帧，在绕 Z 轴 ±20° 范围内以 4° 步长尝试多组初始旋转 + ICP，跳出局部最优（如后腿叠头）。
- 之后再执行 1 轮 LOO 巩固。

### 6. 融合与后处理


| 步骤     | 算法                           | 作用         |
| ------ | ---------------------------- | ---------- |
| 体素融合   | Voxel Grid (0.012m)          | 合并所有帧为统一点云 |
| Z 直通滤波 | PassThrough [0.12, 2.0]m     | 去除地面残余与天花板 |
| 最大簇提取  | Euclidean Cluster Extraction | 分离主体与离散碎团  |
| 双离群滤波  | SOR + Radius Outlier Removal | 削飞点与孤立碎屑   |


### 7. GP3 网格重建

融合点云先体素降采样（0.015m），估计法线后执行 **Greedy Projection Triangulation**。相比 Poisson 重建，GP3 在配准略有残差时不会产生封闭壳伪影，更适合本场景。

## 构建与使用

### 依赖

- **PCL** ≥ 1.10（common, io, filters, registration, kdtree, search, surface, segmentation, features）
- **Eigen**（通过 PCL 间接依赖）
- **C++17**（`std::filesystem`）
- **CMake** ≥ 3.16

### 编译

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### 运行

```bash
./build/sheep_reconstruct --dataset <点云目录> --output <输出目录>
```

- `--dataset`：包含 `.pcd` 文件的目录，递归搜索（默认 `dataset`）
- `--output`：输出目录（默认 `output`）

### 参数调整

所有算法参数在 `[include/sheep_recon/types.hpp](include/sheep_recon/types.hpp)` 的 `ReconParams` 中定义，按场景可调关键项：


| 参数                          | 默认值   | 说明                     |
| --------------------------- | ----- | ---------------------- |
| `voxel_leaf`                | 0.005 | 预处理降采样体素边长(m)，越小越精细但越慢 |
| `icp_corr_dist`             | 0.04  | ICP 精配对应距离(m)          |
| `loo_refine_iters`          | 2     | LOO 细化轮数               |
| `consensus_nn_max_dist`     | 0.038 | 共识裁剪距离阈值(m)            |
| `multistart_yaw_step_deg`   | 4     | Yaw 搜索步长(°)            |
| `multistart_yaw_half_steps` | 5     | Yaw 搜索半步数（±20°）        |
| `fuse_voxel`                | 0.012 | 融合体素边长(m)              |
| `cluster_tolerance`         | 0.028 | 欧式聚类距离阈值(m)            |
| `mesh_voxel`                | 0.015 | 网格输入降采样体素(m)           |
| `gpt_search_radius`         | 0.055 | GP3 搜索半径(m)            |


### 输出文件


| 文件                       | 说明                |
| ------------------------ | ----------------- |
| `output/fused.pcd`       | 配准融合后的点云（二进制 PCD） |
| `output/mesh_greedy.ply` | GP3 三角网格          |
| `output/metrics.txt`     | 分阶段耗时与配准质量指标      |


## 效果

### 运行指标（24 帧示例）

```
$ ./build/sheep_reconstruct --dataset dataset --output output

=== wall_time_ms ===
total               93328
preprocess_frames   24074
sequential_icp      21303
loo_refine          30042
yaw_multistart       7327
mesh_greedy           203

=== registration_quality ===
consensus_nn_mean   0.009 m
consensus_nn_max    0.016 m
consensus_nn_min    0.008 m

=== reconstruction_io ===
fused_points        35684
mesh_triangles      32452
```

- `consensus_nn_*`：各帧到 LOO 共识目标的平均最近邻距离，**越小配准越准**。
- 完整分项耗时与指标自动写入 `output/metrics.txt`。

## 工程结构

```
include/sheep_recon/
├── types.hpp          # 类型别名 + ReconParams 全参数默认值
├── preprocess.hpp     # 单帧预处理（SOR → MLS → 体素 → RANSAC）
├── registration.hpp   # 配准（ICP链 + ELCH + LOO + 共识裁剪 + Yaw搜索）
├── cluster.hpp        # 欧式聚类提取最大簇
├── fuse_mesh.hpp      # 融合 / 滤波 / GP3 网格重建
└── metrics.hpp        # 计时 + 配准质量指标 + 报告输出
src/
└── main.cpp           # 参数解析与流程编排
```

