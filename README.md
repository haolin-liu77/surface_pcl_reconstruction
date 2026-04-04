# 羊多视角点云配准与表面重建

基于 PCL 的离线 C++ 流水线：多帧 RGB 点云 → 配准对齐 → 融合去噪 → 贪婪投影三角网格，输出 `fused.pcd` 与 `mesh_greedy.ply`。

## 技术路径概述

1. **数据加载**：递归收集 `dataset/**/*.pcd`，按文件名排序（与采集序号一致）。
2. **单帧预处理**（每帧独立）：统计离群（SOR）→ 移动最小二乘平滑（MLS）→ 体素下采样 → RANSAC 平面分割去地面，得到多帧「仅羊体为主」的点云。
3. **粗到精链式配准**：相邻帧双阶段 ICP（先大对应距离、再精配），链式累积到第 0 帧坐标系。
4. **ELCH 闭环优化**：将全部帧作为闭环图，减轻顺序 ICP 的累积误差。
5. **Leave-One-Out 细化**：每帧相对「其余帧融合体素靶」再做 ICP，纠正单帧偏离。
6. **共识裁剪**：剔除相对共识表面过远的点，削弱错位层与悬空几何。
7. **最差帧 Yaw 多初值**：对「与共识偏差最大」的一帧，在绕竖轴的一组角度上重试 ICP，减轻局部最优导致的头腿重叠。
8. **短轮 LOO 巩固**：上述步骤后再做少量 LOO 迭代。
9. **融合与后处理**：体素融合 → 可选 Z 直通滤波 → **欧式聚类保留最大簇** → 融合点云上 SOR + 半径离群（ROR）→ 输出 `fused.pcd`。
10. **网格重建**：融合点云体素下采样 → 法线估计 → **Greedy Projection Triangulation（GP3）** → `mesh_greedy.ply`。

工程结构：**逻辑在 `include/sheep_recon/*.hpp` 解耦**，`src/main.cpp` 负责参数与流程编排；可选 `--dataset`、`--output`。

## 效果（示例运行）

一次完整运行示例（24 帧，硬件与环境以实际机器为准）：

```text
$ ./build/sheep_reconstruct --dataset dataset --output output
loaded 24 pcd(s)
wrote output/fused.pcd
wrote output/mesh_greedy.ply
=== wall_time_ms ===
total 93328.633911
load_pcfs 47.373792
preprocess_frames 24074.041776
sequential_icp 21303.406471
apply_transforms 1.924851
elch 2112.456705
loo_refine 30042.201911
strip_consensus 658.630325
yaw_multistart 7327.834451
loo_after_consensus 6628.811671
fuse_voxel 22.208793
passthrough_z 0.208069
cluster_largest 83.259008
fused_sor_ror 123.558048
mesh_voxel_down 1.089193
mesh_greedy 203.625920
save_fused_pcd 0.500892
save_mesh_ply 57.711476
=== registration_quality (lower mean/max NN to LOO consensus is better) ===
frames 24
consensus_nn_mean_m 0.009014
consensus_nn_max_m 0.016496
consensus_nn_min_m 0.008191
=== reconstruction_io ===
fused_points 35684
mesh_input_points_xyz 17063
mesh_triangles 32452
```

- **配准质量**：`consensus_nn_*` 为各帧到 LOO 共识目标的平均最近距离（米）；**数值越小越好**。
- **重建规模**：融合点数、网格输入下采样点数、三角面片数见 `reconstruction_io`；完整分项耗时见 `wall_time_ms`，并写入 `output/metrics.txt`。

### 动图

![](./images/24帧点云配准.gif)

​										   24帧点云重建

![附带RGB的羊重建](./images/附带RGB的羊重建.gif)

​											      点云重建



## 创新点（相对「固定 4 视角 + 单次 ICP + Poisson」类流程）

| 方向 | 说明 |
|------|------|
| **N 视角泛化** | 不限于 04/06/08/10 四帧，按数据集自动适配帧数（如 06–27 共 22 个视角等）。 |
| **LOO + 共识层** | 每帧对齐「除自身外的融合靶」，再走共识距离裁剪，专治某一帧小角度错位、叠层。 |
| **最差帧 Yaw 多初值** | 自动找出与共识最不一致的帧，在竖轴旋转的多组初值上重试 ICP，减轻后腿叠头等局部最优。 |
| **融合后聚类 + 双离群** | 最大簇分离主体与地面碎团；SOR+ROR 进一步削飞点，且已移除「整云二次 RANSAC 去地」以避免误删躯干。 |
| **GP3 网格** | 采用贪婪投影三角化，避免闭环配准略差时 Poisson 易产生难看封闭壳的问题（可按需对比）。 |
| **可复现指标** | 分阶段耗时、共识 NN 指标、点数与面数写入终端与 `metrics.txt`，便于调参与对比实验。 |

## 原始数据集特点与数量

- **数量（当前示例）**：`dataset` 下 **24 个** `.pcd`（含多视角序列，如 `out_captured0406.pcd` … `out_captured0427.pcd` 及少量重复/测试文件时总数可能变化，以实际目录为准）。
- **内容**：每帧主要为 **羊体 + 地面**；视角沿序号递增，**观测方位角依次变化**（转台式采集），适合链式 ICP + 闭环（ELCH）。
- **点类型**：`PointXYZRGB`（PCD 中含 `rgb` 字段）。

## 构建与运行

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/sheep_reconstruct --dataset dataset --output output
```

依赖：PCL（common, io, filters, registration, kdtree, search, surface, segmentation, features 等）、C++17、Eigen。

## 输出文件

| 路径 | 说明 |
|------|------|
| `output/fused.pcd` | 配准并后处理后的融合点云（二进制 PCD） |
| `output/mesh_greedy.ply` | GP3 三角网格 |
| `output/metrics.txt` | 与终端一致的性能与质量指标 |

算法默认参数见 [`include/sheep_recon/types.hpp`](include/sheep_recon/types.hpp)。
