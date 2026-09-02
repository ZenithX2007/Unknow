/**
 * @file ssc_map.cc
 * @author GW
 * @brief SSC地图核心实现：时空语义立方体地图的构建和管理
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 * 
 * **文件概述**：
 * 本文件实现了SSC（Spatio-temporal Semantic Corridor）地图的核心功能，
 * 对应论文《基于时空语义走廊的复杂城市环境安全轨迹生成》中的SSC地图构建算法。
 * 
 * **核心功能**（对应文档章节）：
 * 1. **SSC地图构建**（文档第5.1节：语义元素与Frenét坐标系表示）
 *    - FillStaticPart()：填充静态障碍物（跨越整个时间轴）
 *    - FillDynamicPart()：填充动态障碍物（时间域中的一系列静态障碍物）
 *    - 生成三维占据网格（slt域）
 * 
 * 2. **语义走廊生成**（文档第5.2节：语义走廊生成算法）
 *    - ConstructCorridorUsingInitialTrajectory()：实现算法1的核心流程
 *      - 种子生成（算法1第3行）
 *      - 立方体膨胀（算法1第4行）
 *      - 约束关联（算法1第5行）
 *      - 立方体松弛（算法1第6行）
 * 
 * 3. **立方体膨胀**（文档第5.2节：带语义边界的立方体膨胀）
 *    - InflateCubeIn3dGrid()：在slt三维空间中膨胀立方体
 *    - InflateCubeOnX/Y/ZPos/NegAxis()：各方向的膨胀操作
 *    - 处理语义边界和障碍物的碰撞检测
 * 
 * 4. **立方体松弛**（文档第5.2节：立方体松弛）
 *    - CorridorRelaxation()：在满足硬约束的前提下松弛立方体边界
 *    - 为优化过程预留额外空间
 * 
 * **关键数据结构**：
 * - **三维占据网格**（GridMap3D）：slt三维配置空间的占据网格
 *   - 0：自由空间
 *   - 100：障碍物占据
 * - **驱动走廊**（DrivingCorridor）：由一系列时空立方体组成的走廊
 * - **时空语义立方体**（SpatioTemporalSemanticCubeNd）：包含位置、速度、加速度约束的立方体
 * 
 * **坐标系**：
 * - **slt三维配置空间**：
 *   - s（纵向）：沿参考车道的弧长距离
 *   - l（横向）：垂直于参考车道方向的偏移距离
 *   - t（时间）：时间维度，用于处理动态障碍物
 * 
 * **参考文档**：paper/SSC.md 第5节（时空语义走廊）
 */

#include "ssc_planner/ssc_map.h"

#include <glog/logging.h>

namespace planning {

/**
 * @brief SSC地图构造函数
 * @param config 地图配置参数（尺寸、分辨率、动力学边界等）
 * 
 * **功能**：
 * 初始化SSC地图对象，创建三维占据网格和膨胀网格。
 * 
 * **创建的网格**：
 * 1. **p_3d_grid_**：三维占据网格（用于存储障碍物信息）
 *    - 维度：slt三个维度（纵向s、横向l、时间t）
 *    - 分辨率：config_.map_resolution
 *    - 尺寸：config_.map_size
 * 
 * 2. **p_3d_inflated_grid_**：膨胀后的三维网格（可选，用于障碍物膨胀）
 *    - 尺寸和分辨率与p_3d_grid_相同
 *    - 用于存储膨胀后的障碍物信息（增加安全裕量）
 * 
 * **配置参数**（对应文档第5节：SSC地图配置）：
 * - map_size[0,1,2]：slt三个维度的地图尺寸
 * - map_resolution[0,1,2]：slt三个维度的地图分辨率
 * - 动力学边界：速度、加速度的上下界（用于后续立方体约束）
 * 
 * **对应文档**：第5节（时空语义走廊）
 */
SscMap::SscMap(const SscMap::Config &config) : config_(config) {
  // 打印配置信息（用于调试和验证）
  config_.Print();

  // 创建三维占据网格（slt三维配置空间）
  // 用于存储静态和动态障碍物的占据信息
  p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
  // 创建膨胀后的三维网格（可选，用于障碍物膨胀）
  // 当前EPSILON项目中通常不使用膨胀网格
  p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
}

/**
 * @brief 重置SSC地图（用于新的规划周期）
 * @param ini_fs 初始Frenet状态（用于设置地图原点）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 清空已有的走廊和地图数据，并使用新的初始状态重置地图原点。
 * 
 * **重置内容**：
 * 1. **清空驱动走廊**：ClearDrivingCorridor() - 清空之前生成的走廊
 * 2. **清空地图数据**：ClearGridMap() - 清空三维占据网格
 * 3. **设置时间原点**：start_time_ = ini_fs.time_stamp
 * 4. **更新地图原点**：UpdateMapOrigin() - 基于新的初始状态设置地图原点
 * 
 * **地图原点设置**（UpdateMapOrigin）：
 * - 纵向s：ori_fs.vec_s[0] - config_.s_back_len（向后延伸以处理历史状态）
 * - 横向l：-(map_size[1] - 1) * map_resolution[1] / 2.0（居中）
 * - 时间t：ori_fs.time_stamp（当前时间戳）
 * 
 * **调用时机**：
 * - 每个规划周期开始时调用（RunOnce()中调用）
 * - 确保地图数据与当前规划周期对齐
 * 
 * **对应文档**：第5节（时空语义走廊）
 */
ErrorType SscMap::ResetSscMap(const common::FrenetState &ini_fs) {
  // 清空之前生成的驱动走廊
  ClearDrivingCorridor();
  // 清空三维占据网格数据
  ClearGridMap();

  // 设置时间原点（用于将全局时间转换为相对时间）
  start_time_ = ini_fs.time_stamp;
  // 更新地图原点（基于新的初始Frenet状态）
  UpdateMapOrigin(ini_fs);

  return kSuccess;
}

/**
 * @brief 更新地图原点（基于初始Frenet状态）
 * @param ori_fs 参考Frenet状态（用于计算地图原点）
 * 
 * **功能**：
 * 根据给定的Frenet状态计算并设置地图原点，使地图坐标系与当前规划状态对齐。
 * 
 * **地图原点计算**（slt三维配置空间）：
 * 
 * 1. **纵向s维度**：map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len
 *    - ori_fs.vec_s[0]：初始状态的纵向位置
 *    - s_back_len：向后延伸长度（用于处理历史状态）
 *    - **物理意义**：地图起点在当前自车位置后方s_back_len距离处
 * 
 * 2. **横向l维度**：map_origin[1] = -(map_size[1] - 1) * map_resolution[1] / 2.0
 *    - **物理意义**：横向维度居中设置，使地图在横向方向对称
 *    - 地图范围：[-map_size/2 * resolution, +map_size/2 * resolution]
 * 
 * 3. **时间t维度**：map_origin[2] = ori_fs.time_stamp
 *    - **物理意义**：地图时间起点为当前规划周期的时间戳
 *    - 用于将全局时间转换为相对于规划起点的相对时间
 * 
 * **用途**：
 * - 确保地图坐标系与当前规划状态对齐
 * - 方便后续的障碍物填充和走廊生成
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
  // 保存初始Frenet状态（用于后续的走廊生成）
  initial_fs_ = ori_fs;

  // 计算地图原点（slt三维配置空间）
  std::array<decimal_t, 3> map_origin;
  // 纵向s维度：向后延伸s_back_len（处理历史状态）
  map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;
  // 横向l维度：居中设置（使地图在横向方向对称）
  map_origin[1] =
      -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;  // d
  // 时间t维度：设置为当前时间戳（规划周期的起点）
  map_origin[2] = ori_fs.time_stamp;                                     // t

  // 设置三维占据网格的原点
  p_3d_grid_->set_origin(map_origin);
  // 设置膨胀网格的原点（与占据网格一致）
  p_3d_inflated_grid_->set_origin(map_origin);
}

/**
 * @brief 使用两个种子生成初始立方体（对应文档算法1：种子生成）
 * @param seed_0 第一个种子（网格坐标）
 * @param seed_1 第二个种子（网格坐标）
 * @param cube 输出：初始立方体（轴对齐的立方体）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 根据两个连续的种子状态生成初始立方体，作为立方体膨胀的起点。
 * 
 * **初始立方体生成**（对应文档算法2第5行）：
 * - 立方体的下界（lb）：两个种子在各维度上的最小值
 *   - lb[0] = min(seed_0(0), seed_1(0))：纵向s的最小值
 *   - lb[1] = min(seed_0(1), seed_1(1))：横向l的最小值
 *   - lb[2] = min(seed_0(2), seed_1(2))：时间t的最小值
 * 
 * - 立方体的上界（ub）：两个种子在各维度上的最大值
 *   - ub[0] = max(seed_0(0), seed_1(0))：纵向s的最大值
 *   - ub[1] = max(seed_0(1), seed_1(1))：横向l的最大值
 *   - ub[2] = max(seed_0(2), seed_1(2))：时间t的最大值
 * 
 * **物理意义**：
 * - 初始立方体连接两个连续的种子状态
 * - 该立方体是立方体膨胀的起点，将在后续步骤中膨胀到障碍物边界
 * - 确保初始立方体包含两个种子状态
 * 
 * **对应文档**：第5.2节（语义走廊生成算法）、算法2（CubeInflation）
 */
ErrorType SscMap::GetInitialCubeUsingSeed(
    const Vec3i &seed_0, const Vec3i &seed_1,
    common::AxisAlignedCubeNd<int, 3> *cube) const {
  // 计算立方体下界（各维度的最小值）
  std::array<int, 3> lb;
  lb[0] = std::min(seed_0(0), seed_1(0));  // 纵向s的最小值
  lb[1] = std::min(seed_0(1), seed_1(1));  // 横向l的最小值
  lb[2] = std::min(seed_0(2), seed_1(2));  // 时间t的最小值
  
  // 计算立方体上界（各维度的最大值）
  std::array<int, 3> ub;
  ub[0] = std::max(seed_0(0), seed_1(0));  // 纵向s的最大值
  ub[1] = std::max(seed_0(1), seed_1(1));  // 横向l的最大值
  ub[2] = std::max(seed_0(2), seed_1(2));  // 时间t的最大值

  // 创建轴对齐的立方体（用于后续膨胀）
  *cube = common::AxisAlignedCubeNd<int, 3>(ub, lb);
  return kSuccess;
}

/**
 * @brief 构建SSC地图（对应文档第5.1节：类障碍物语义元素）
 * @param sur_vehicle_trajs_fs 周围车辆的前向轨迹（Frenet坐标系）
 * @param obstacle_grids 静态障碍物网格点（Frenet坐标系）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 构建三维占据网格（slt域），将语义元素渲染为障碍物。
 * 
 * **构建流程**（对应文档第5.1节）：
 * 1. **清空地图数据**：清空之前的三维占据网格
 * 2. **填充静态障碍物**：FillStaticPart() - 将静态障碍物渲染为跨越整个时间轴的障碍物
 * 3. **填充动态障碍物**：FillDynamicPart() - 将动态障碍物渲染为时间域中的一系列静态障碍物
 * 
 * **类障碍物语义元素**（文档第5.1节）：
 * 1. **静态障碍物**（FillStaticPart）：
 *    - 可视为跨越整个时间轴的障碍物
 *    - 在slt三维空间中沿时间轴t延伸
 * 
 * 2. **动态障碍物**（FillDynamicPart）：
 *    - 根据预测轨迹视为时间域中的一系列静态障碍物
 *    - 每个时间片对应一个静态障碍物
 * 
 * 3. **交通信号灯**（如红灯）：
 *    - 可渲染为占据特定纵向位置和时间段的障碍物
 *    - 在当前实现中通过其他机制处理（不在本函数中）
 * 
 * **三维占据网格**：
 * - 0：自由空间
 * - 100：障碍物占据
 * - 维度：slt（纵向s、横向l、时间t）
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
ErrorType SscMap::ConstructSscMap(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs,
    const vec_E<Vec2f> &obstacle_grids) {
  // 清空之前的地图数据（为新规划周期准备）
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  
  // 填充静态障碍物（跨越整个时间轴的障碍物）
  FillStaticPart(obstacle_grids);
  // 填充动态障碍物（时间域中的一系列静态障碍物）
  FillDynamicPart(sur_vehicle_trajs_fs);
  return kSuccess;
}

ErrorType SscMap::GetInflationDirections(const bool &if_first_cube,
                                         std::array<bool, 6> *dirs_disabled) {
  (*dirs_disabled)[0] = false;
  (*dirs_disabled)[1] = false;
  (*dirs_disabled)[2] = false;
  (*dirs_disabled)[3] = false;
  (*dirs_disabled)[4] = false;
  (*dirs_disabled)[5] = !if_first_cube;

  return kSuccess;
}

ErrorType SscMap::ClearGridMap() {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  return kSuccess;
}

ErrorType SscMap::ClearDrivingCorridor() {
  driving_corridor_vec_.clear();
  return kSuccess;
}

/**
 * @brief 使用初始轨迹构建语义走廊（对应文档算法1：语义走廊生成）
 * @param p_grid 三维占据网格（包含障碍物信息）
 * @param trajs 前向仿真轨迹（Frenet坐标系，作为种子）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 实现文档第5.2节"语义走廊生成"算法（算法1）的核心流程，生成时空语义立方体序列。
 * 
 * **核心流程**（对应文档算法1）：
 * 1. **种子生成**（算法1第3行）：从前向仿真轨迹中提取种子状态
 * 2. **立方体膨胀**（算法1第4行）：以种子为中心，在slt三维空间中膨胀生成立方体
 * 3. **约束关联**（算法1第5行）：将语义边界与立方体关联（当前实现中通过膨胀策略处理）
 * 4. **立方体松弛**（算法1第6行）：在满足硬约束的前提下，松弛立方体边界（当前已禁用）
 * 
 * **对应文档**：
 * - 第5.2节：语义走廊生成算法（算法1）
 * - 第5.3节：立方体膨胀、约束关联、立方体松弛的具体实现
 * 
 * **输出**：
 * - driving_corridor_vec_：生成的驱动走廊（时空语义立方体序列）
 * - 每个立方体定义了一个时空自由空间区域
 * - 立方体用于后续的轨迹优化（Bézier曲线参数化）
 */
ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
  /**
   * **阶段I：种子生成（对应文档算法1第3行）**
   * 
   * 从前向仿真轨迹中提取种子状态，每个种子对应一个初始立方体的中心。
   * 
   * **种子生成策略**（对应文档算法1第3行）：
   * - 第一个种子：初始状态（initial_fs_）
   * - 后续种子：前向仿真轨迹的状态
   * - 种子转换：将Frenet坐标转换为网格坐标
   * 
   * **种子要求**：
   * - 必须在网格范围内（CheckCoordInRange）
   * - 时间必须晚于起始时间（coord[2] > 0）
   * - 相邻种子需要足够的间隙（文档第5.2节要求）
   */
  // ~ Stage I: Get seeds
  vec_E<Vec3i> traj_seeds;
  int num_states = static_cast<int>(trajs.size());
  if (num_states > 1) {
    bool first_seed_determined = false;
    for (int k = 0; k < num_states; ++k) {
      std::array<decimal_t, 3> p_w = {};
      if (!first_seed_determined) {
        decimal_t s_0 = initial_fs_.vec_s[0];
        decimal_t d_0 = initial_fs_.vec_dt[0];
        decimal_t t_0 = initial_fs_.time_stamp;
        std::array<decimal_t, 3> p_w_0 = {s_0, d_0, t_0};
        auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);

        decimal_t s_1 = trajs[k].frenet_state.vec_s[0];
        decimal_t d_1 = trajs[k].frenet_state.vec_dt[0];
        decimal_t t_1 = trajs[k].frenet_state.time_stamp;
        std::array<decimal_t, 3> p_w_1 = {s_1, d_1, t_1};
        auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);
        // * remove the states out of range
        if (!p_grid->CheckCoordInRange(coord_1)) {
          continue;
        }
        // earlier than start time
        if (coord_1[2] <= 0) {
          continue;
        }

        first_seed_determined = true;
        traj_seeds.push_back(Vec3i(coord_0[0], coord_0[1], coord_0[2]));
        traj_seeds.push_back(Vec3i(coord_1[0], coord_1[1], coord_1[2]));
      } else {
        decimal_t s = trajs[k].frenet_state.vec_s[0];
        decimal_t d = trajs[k].frenet_state.vec_dt[0];
        decimal_t t = trajs[k].frenet_state.time_stamp;
        p_w = {s, d, t};
        auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
        // * remove the states out of range
        if (!p_grid->CheckCoordInRange(coord)) {
          continue;
        }
        traj_seeds.push_back(Vec3i(coord[0], coord[1], coord[2]));
      }
    }
  }

  /**
   * **阶段II：立方体膨胀（对应文档算法1第4行：立方体膨胀）**
   * 
   * 以种子为中心，在slt三维空间中膨胀生成立方体，直到遇到障碍物或语义边界。
   * 
   * **立方体膨胀策略**（对应文档算法2：CubeInflation）：
   * 1. **初始立方体生成**：基于两个连续种子生成初始立方体（GetInitialCubeUsingSeed）
   * 2. **立方体膨胀**：在三个slt方向交替膨胀（InflateCubeIn3dGrid）
   * 3. **终止条件**：
   *    - 遇到障碍物（CheckIfCubeIsFree返回false）
   *    - 遇到语义边界（通过膨胀方向禁用处理）
   *    - 达到动力学边界（纵向膨胀受速度/加速度限制）
   * 
   * **对应文档**：第5.2节（语义走廊生成算法）、算法2（CubeInflation）
   */
  // ~ Stage II: Inflate cubes
  common::DrivingCorridor driving_corridor;
  bool is_valid = true;
  auto seed_num = static_cast<int>(traj_seeds.size());
  
  // 检查种子数量（至少需要2个种子才能生成初始立方体）
  if (seed_num < 2) {
    // 种子数量不足，标记走廊为无效
    driving_corridor.is_valid = false;
    driving_corridor_vec_.push_back(driving_corridor);
    is_valid = false;
    return kWrongStatus;
  }
  // 遍历所有种子，生成立方体序列
  for (int i = 0; i < seed_num; ++i) {
    // 第一个立方体：基于前两个种子生成
    if (i == 0) {
      common::AxisAlignedCubeNd<int, 3> cube;
      // 使用前两个种子生成初始立方体（算法2第5行）
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
      
      // 检查初始立方体是否无碰撞（算法2第6行）
      if (!CheckIfCubeIsFree(p_grid, cube)) {
        LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;

        // 初始立方体有碰撞，标记走廊为无效并返回
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);

        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }

      // 立方体膨胀（算法2第9行）：在slt三个方向膨胀
      // dirs_disabled：六个方向的膨胀禁用标志（默认全部允许）
      // - [0-5]：s+, s-, l+, l-, t+, t-
      std::array<bool, 6> dirs_disabled = {false, false, false,
                                           false, false, false};
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);

      // 保存膨胀后的立方体
      common::DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);
    } else {
      // 后续种子：检查是否已包含在最后一个立方体中
      if (CheckIfCubeContainsSeed(driving_corridor.cubes.back().cube,
                                  traj_seeds[i])) {
        // 种子已在最后一个立方体中，添加到该立方体的种子列表（拓扑等价）
        driving_corridor.cubes.back().seeds.push_back(traj_seeds[i]);
        continue;
      } else {
        // 种子不在最后一个立方体中，需要生成新立方体
        // 获取最后一个立方体的最后一个种子（用于切割时间轴）
        Vec3i seed_r = driving_corridor.cubes.back().seeds.back();
        driving_corridor.cubes.back().seeds.pop_back();
        // 在时间轴上切割立方体（使相邻立方体时间连续，对应文档公式8的连续性要求）
        driving_corridor.cubes.back().cube.upper_bound[2] = seed_r(2);
        i = i - 1;  // 回退一个索引，重新处理当前种子

        // 使用当前种子和下一个种子生成新的初始立方体
        common::AxisAlignedCubeNd<int, 3> cube;
        GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);

        // 检查初始立方体是否无碰撞
        if (!CheckIfCubeIsFree(p_grid, cube)) {
          LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: "
                     << i;
          // 初始立方体有碰撞，标记走廊为无效并返回
          common::DrivingCube driving_cube;
          driving_cube.cube = cube;
          driving_cube.seeds.push_back(traj_seeds[i]);
          driving_cube.seeds.push_back(traj_seeds[i + 1]);
          driving_corridor.cubes.push_back(driving_cube);

          driving_corridor.is_valid = false;
          driving_corridor_vec_.push_back(driving_corridor);
          is_valid = false;
          break;
        }

        // 立方体膨胀（在slt三个方向膨胀）
        std::array<bool, 6> dirs_disabled = {false, false, false,
                                             false, false, false};
        InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps,
                            &cube);
        // 保存膨胀后的立方体
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_corridor.cubes.push_back(driving_cube);
      }
    }
  }
  
  // 如果走廊生成成功，进行最终处理
  if (is_valid) {
    // 立方体松弛（算法1第6行，当前已禁用）
    // CorridorRelaxation(p_grid, &driving_corridor);
    
    // 在时间轴上切割最后一个立方体（使其时间上界与最后一个种子对齐）
    // 确保相邻立方体时间连续（对应文档公式8的连续性要求）
    driving_corridor.cubes.back().cube.upper_bound[2] = traj_seeds.back()(2);
    
    // 标记走廊为有效并保存
    driving_corridor.is_valid = true;
    driving_corridor_vec_.push_back(driving_corridor);
  }

  return kSuccess;
}

ErrorType SscMap::GetTimeCoveredCubeIndices(
    const common::DrivingCorridor *p_corridor, const int &start_idx,
    const int &dir, const int &t_trans, std::vector<int> *idx_list) const {
  int dt = 0;
  int num_cube = p_corridor->cubes.size();
  int idx = start_idx;
  while (idx < num_cube && idx >= 0) {
    dt += p_corridor->cubes[idx].cube.upper_bound[2] -
          p_corridor->cubes[idx].cube.lower_bound[2];
    idx_list->push_back(idx);
    if (dir == 1) {
      ++idx;
    } else {
      --idx;
    }
    if (dt >= t_trans) {
      break;
    }
  }
  return kSuccess;
}

ErrorType SscMap::CorridorRelaxation(GridMap3D *p_grid,
                                     common::DrivingCorridor *p_corridor) {
  std::array<int, 2> margin = {{50, 10}};
  int t_trans = 7;
  int num_cube = p_corridor->cubes.size();
  for (int i = 0; i < num_cube - 1; ++i) {
    if (1)  // ~ Enable s direction
    {
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[0];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[0];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[0];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[0];

      if (abs(cube_0_ub - cube_1_lb) < margin[0]) {
        int room = margin[0] - abs(cube_0_ub - cube_1_lb);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[0]) {
        int room = margin[0] - abs(cube_0_lb - cube_1_ub);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }

    if (1)  // ~ Enable d direction
    {
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[1];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[1];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[1];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[1];

      if (abs(cube_0_ub - cube_1_lb) < margin[1]) {
        int room = margin[1] - abs(cube_0_ub - cube_1_lb);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[1]) {
        int room = margin[1] - abs(cube_0_lb - cube_1_ub);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto idx : up_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto idx : down_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::InflateObstacleGrid(const common::VehicleParam &param) {
  decimal_t s_p_inflate_len = param.length() / 2.0 - param.d_cr();
  decimal_t s_n_inflate_len = param.length() - s_p_inflate_len;
  int num_s_p_inflate_grids =
      std::floor(s_p_inflate_len / config_.map_resolution[0]);
  int num_s_n_inflate_grids =
      std::floor(s_n_inflate_len / config_.map_resolution[0]);
  int num_d_inflate_grids =
      std::floor((param.width() - 0.5) / 2.0 / config_.map_resolution[1]);
  bool is_free = false;

  for (int i = 0; i < config_.map_size[0]; ++i) {
    for (int j = 0; j < config_.map_size[1]; ++j) {
      for (int k = 0; k < config_.map_size[2]; ++k) {
        std::array<int, 3> coord = {i, j, k};
        p_3d_grid_->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          for (int s = -num_s_n_inflate_grids; s < num_s_p_inflate_grids; s++) {
            for (int d = -num_d_inflate_grids; d < num_d_inflate_grids; d++) {
              coord = {i + s, j + d, k};
              p_3d_inflated_grid_->SetValueUsingCoordinate(coord, 100);
            }
          }
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::InflateCubeIn3dGrid(GridMap3D *p_grid,
                                      const std::array<bool, 6> &dir_disabled,
                                      const std::array<int, 6> &dir_step,
                                      common::AxisAlignedCubeNd<int, 3> *cube) {
  // Each cube is expanded only through free slt cells.  The additional
  // longitudinal bounds below are a reachable-set approximation: even empty
  // cells outside the acceleration envelope cannot belong to this corridor.
  bool x_p_finish = dir_disabled[0];
  bool x_n_finish = dir_disabled[1];
  bool y_p_finish = dir_disabled[2];
  bool y_n_finish = dir_disabled[3];
  bool z_p_finish = dir_disabled[4];

  int x_p_step = dir_step[0];
  int x_n_step = dir_step[1];
  int y_p_step = dir_step[2];
  int y_n_step = dir_step[3];
  int z_p_step = dir_step[4];

  int t_max_grids = cube->lower_bound[2] + config_.kMaxNumOfGridAlongTime;

  decimal_t t = t_max_grids * p_grid->dims_resolution(2);
  decimal_t a_max = config_.kMaxLongitudinalAcc;
  decimal_t a_min = config_.kMaxLongitudinalDecel;
  decimal_t d_comp = initial_fs_.vec_s[1] * 1;

  // Reachable longitudinal interval under maximum acceleration/deceleration.
  // It keeps the corridor compatible with the derivative constraints later
  // imposed on the corresponding Bezier segment.
  decimal_t s_u = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_max * t * t + d_comp;
  decimal_t s_l = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_min * t * t - d_comp;

  int s_idx_u, s_idx_l;
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);
  s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len / 2.0) /
                                               config_.map_resolution[0]));

  while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
    if (!x_p_finish) x_p_finish = InflateCubeOnXPosAxis(p_grid, x_p_step, cube);
    if (!x_n_finish) x_n_finish = InflateCubeOnXNegAxis(p_grid, x_n_step, cube);

    if (!y_p_finish) y_p_finish = InflateCubeOnYPosAxis(p_grid, y_p_step, cube);
    if (!y_n_finish) y_n_finish = InflateCubeOnYNegAxis(p_grid, y_n_step, cube);

    if (cube->upper_bound[0] >= s_idx_u) x_p_finish = true;
    if (cube->lower_bound[0] <= s_idx_l) x_n_finish = true;
  }

  // ~ No need to inflate along z-neg
  while (!z_p_finish) {
    if (!z_p_finish) z_p_finish = InflateCubeOnZPosAxis(p_grid, z_p_step, cube);

    if (cube->upper_bound[2] - cube->lower_bound[2] >=
        config_.kMaxNumOfGridAlongTime) {
      z_p_finish = true;
    }
  }

  return kSuccess;
}

bool SscMap::InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->upper_bound[0] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        // The plane in 3D obstacle grid is free
        cube->upper_bound[0] = x;
      } else {
        // The plane in 3D obstacle grid is not free, finish
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->lower_bound[0] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        // The plane in 3D obstacle grid is free
        cube->lower_bound[0] = x;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->upper_bound[1] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        // The plane in 3D obstacle grid is free
        cube->upper_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->lower_bound[1] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        // The plane in 3D obstacle grid is free
        cube->lower_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->upper_bound[2] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        // The plane in 3D obstacle grid is free
        cube->upper_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->lower_bound[2] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        // The plane in 3D obstacle grid is free
        cube->lower_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  int f2_min = cube.lower_bound[2];
  int f2_max = cube.upper_bound[2];

  int i, j, k;
  std::array<int, 3> coord;
  bool is_free;
  for (i = f0_min; i <= f0_max; ++i) {
    for (j = f1_min; j <= f1_max; ++j) {
      for (k = f2_min; k <= f2_max; ++k) {
        coord = {i, j, k};
        p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          return false;
        }
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnXAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &x) const {
  int f0_min = cube.lower_bound[1];
  int f0_max = cube.upper_bound[1];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {x, i, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnYAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &y) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, y, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnZAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &z) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, j, z};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfCubeContainsSeed(
    const common::AxisAlignedCubeNd<int, 3> &cube_a, const Vec3i &seed) const {
  for (int i = 0; i < 3; ++i) {
    if (cube_a.lower_bound[i] > seed(i) || cube_a.upper_bound[i] < seed(i)) {
      return false;
    }
  }
  return true;
}

ErrorType SscMap::GetFinalGlobalMetricCubesList() {
  // The grid is convenient for collision tests, whereas the QP consumes
  // metric s/l/t boxes plus derivative bounds.  This conversion is the handoff
  // between SSC construction and the Bezier optimization described in SSC.md.
  final_corridor_vec_.clear();
  if_corridor_valid_.clear();
  for (const auto corridor : driving_corridor_vec_) {
    vec_E<common::SpatioTemporalSemanticCubeNd<2>> cubes;
    if (!corridor.is_valid) {
      if_corridor_valid_.push_back(0);
    } else {
      if_corridor_valid_.push_back(1);
      for (int k = 0; k < static_cast<int>(corridor.cubes.size()); ++k) {
        common::SpatioTemporalSemanticCubeNd<2> cube;
        decimal_t x_lb, x_ub;
        decimal_t y_lb, y_ub;
        decimal_t z_lb, z_ub;

        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[0], 0, &x_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[0], 0, &x_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[1], 1, &y_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[1], 1, &y_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[2], 2, &z_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[2], 2, &z_ub);

        cube.t_lb = z_lb;
        cube.t_ub = z_ub;

        // Position, velocity and acceleration bounds become sufficient
        // constraints on the Bezier control points of this corridor segment.
        cube.p_lb[0] = x_lb;
        cube.p_ub[0] = x_ub;
        cube.v_lb[0] = config_.kMinLongitudinalVel;
        cube.v_ub[0] = config_.kMaxLongitudinalVel;
        cube.a_lb[0] = config_.kMaxLongitudinalDecel;
        cube.a_ub[0] = config_.kMaxLongitudinalAcc;

        cube.p_lb[1] = y_lb;
        cube.p_ub[1] = y_ub;
        cube.v_lb[1] = -config_.kMaxLateralVel;
        cube.v_ub[1] = config_.kMaxLateralVel;
        cube.a_lb[1] = -config_.kMaxLateralAcc;
        cube.a_ub[1] = config_.kMaxLateralAcc;

        if (k == 0) {
          if (y_lb > initial_fs_.vec_dt[0] || y_ub < initial_fs_.vec_dt[0]) {
            LOG(ERROR) << "[Ssc] SscMap - Initial state out of bound d: "
                       << initial_fs_.vec_dt[0] << ", lb: " << y_lb
                       << ", ub: " << y_ub;
            // assert(false);
            return kWrongStatus;
          }
        }

        cubes.push_back(cube);
      }
    }
    final_corridor_vec_.push_back(cubes);
  }

  return kSuccess;
}

/**
 * @brief 填充静态障碍物（对应文档第5.1节：类障碍物语义元素）
 * @param obs_grid_fs 静态障碍物网格点（Frenet坐标系，slt中的sl坐标）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 将静态障碍物渲染为跨越整个时间轴的障碍物（对应文档第5.1节）。
 * 
 * **静态障碍物处理**（对应文档第5.1节）：
 * - **特征**：静态障碍物可视为跨越整个时间轴的障碍物
 * - **渲染方式**：在slt三维空间中，沿时间轴t延伸整个地图高度
 *   - 对于每个障碍物网格点(s, l)
 *   - 在时间维度t上从0到map_size[2]全部标记为障碍物（值为100）
 * 
 * **三维占据网格**：
 * - 0：自由空间
 * - 100：障碍物占据
 * 
 * **处理流程**：
 * 1. 遍历所有静态障碍物网格点
 * 2. 对于每个网格点(s, l)，在时间轴t上全部标记为障碍物
 * 3. 将Frenet坐标(s, l, t)转换为网格坐标并设置障碍物值
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
ErrorType SscMap::FillStaticPart(const vec_E<Vec2f> &obs_grid_fs) {
  // 遍历所有静态障碍物网格点
  for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
    // 跳过无效的障碍物点（纵向坐标s <= 0）
    if (obs_grid_fs[i](0) <= 0) {
      continue;
    }
    // 在时间轴t上延伸整个地图高度（静态障碍物跨越整个时间轴）
    for (int k = 0; k < config_.map_size[2]; ++k) {
      // 构建三维点(s, l, t)：静态障碍物在时间维度上的位置
      std::array<decimal_t, 3> pt = {{obs_grid_fs[i](0), obs_grid_fs[i](1),
                                      (double)k * config_.map_resolution[2]}};
      // 将Frenet坐标转换为网格坐标
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
      // 检查坐标是否在网格范围内
      if (p_3d_grid_->CheckCoordInRange(coord)) {
        // 设置障碍物值（100表示障碍物占据）
        p_3d_grid_->SetValueUsingCoordinate(coord, 100);
      }
    }
  }
  return kSuccess;
}

/**
 * @brief 填充动态障碍物（对应文档第5.1节：类障碍物语义元素）
 * @param sur_vehicle_trajs_fs 周围车辆的前向轨迹（Frenet坐标系）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 将动态障碍物（其他车辆的预测轨迹）渲染为时间域中的一系列静态障碍物。
 * 
 * **动态障碍物处理**（对应文档第5.1节）：
 * - **特征**：动态障碍物可根据预测轨迹视为时间域中的一系列静态障碍物
 * - **渲染方式**：对于每个时间片，将车辆轮廓作为静态障碍物
 *   - 每个时间片对应一个静态障碍物
 *   - 车辆轮廓通过fillPoly填充到占据网格中
 * 
 * **处理流程**：
 * 1. 遍历所有周围车辆的前向轨迹
 * 2. 对于每辆车，调用FillMapWithFsVehicleTraj()填充其轨迹
 * 3. 在每个时间片，将车辆轮廓（顶点）填充到占据网格中
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
ErrorType SscMap::FillDynamicPart(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs) {
  // 遍历所有周围车辆的前向轨迹
  for (auto it = sur_vehicle_trajs_fs.begin(); it != sur_vehicle_trajs_fs.end();
       ++it) {
    // 填充每辆车的前向轨迹（在每个时间片将车辆轮廓标记为障碍物）
    FillMapWithFsVehicleTraj(it->second);
  }
  return kSuccess;
}

/**
 * @brief 使用车辆前向轨迹填充地图（对应文档第5.1节：动态障碍物）
 * @param traj 车辆的前向轨迹（Frenet坐标系，包含每个时间片的车辆轮廓）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 将车辆的前向轨迹填充到三维占据网格中，每个时间片对应一个车辆轮廓。
 * 
 * **动态障碍物渲染**（对应文档第5.1节）：
 * - 对于轨迹中的每个状态（时间片）
 * - 将车辆轮廓（4个顶点）填充到占据网格的对应时间层
 * - 使用OpenCV的fillPoly函数填充车辆占据区域
 * 
 * **处理流程**：
 * 1. **轨迹验证**：检查轨迹是否为空
 * 2. **时间片处理**：遍历轨迹中的每个状态
 *    - 获取车辆轮廓顶点（Frenet坐标系）
 *    - 将顶点转换为网格坐标
 *    - 获取对应的时间层索引
 * 3. **轮廓填充**：使用fillPoly将车辆轮廓填充到时间层
 *    - 转换顶点坐标（从common::Point2i到cv::Point2i）
 *    - 创建时间层的Mat视图
 *    - 使用fillPoly填充车辆占据区域（值为100）
 * 
 * **三维占据网格结构**：
 * - 数据按时间层存储：layer_offset = t_idx * width * height
 * - 每个时间层对应一个二维占据网格（slt中的sl平面）
 * - 车辆轮廓填充到对应时间层的sl平面
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
ErrorType SscMap::FillMapWithFsVehicleTraj(
    const vec_E<common::FsVehicle> traj) {
  // 检查轨迹是否为空
  if (traj.size() == 0) {
    LOG(ERROR) << "[Ssc] SscMap - Trajectory is empty.";
    return kWrongStatus;
  }
  
  // 遍历轨迹中的每个状态（每个状态对应一个时间片）
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    // 验证车辆轮廓顶点是否有效（纵向坐标s > 0）
    bool is_valid = true;
    for (const auto v : traj[i].vertices) {
      if (v(0) <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }
    
    // 获取当前状态的时间戳
    decimal_t z = traj[i].frenet_state.time_stamp;
    int t_idx = 0;  // 时间层索引
    std::vector<common::Point2i> v_coord;  // 车辆轮廓的网格坐标
    
    // 将车辆轮廓顶点转换为网格坐标
    std::array<decimal_t, 3> p_w;
    for (const auto v : traj[i].vertices) {
      // 构建三维点(s, l, t)：车辆轮廓顶点在Frenet坐标系中的位置
      p_w = {v(0), v(1), z};
      // 将Frenet坐标转换为网格坐标
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      // 获取时间层索引（第三个维度）
      t_idx = coord[2];
      // 检查坐标是否在网格范围内
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      // 保存车辆轮廓的网格坐标（sl平面的坐标，忽略时间维度）
      v_coord.push_back(common::Point2i(coord[0], coord[1]));
    }
    if (!is_valid) {
      continue;
    }
    
    // 将车辆轮廓坐标转换为OpenCV格式
    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    std::vector<cv::Point2i> v_coord_cv;
    common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord,
                                                             &v_coord_cv);
    vv_coord_cv.push_back(v_coord_cv);
    
    // 获取时间层的Mat视图（用于fillPoly填充）
    int w = p_3d_grid_->dims_size()[0];  // 纵向s的网格数
    int h = p_3d_grid_->dims_size()[1];  // 横向l的网格数
    int layer_offset = t_idx * w * h;    // 时间层的偏移量
    // 创建时间层的Mat视图（直接操作网格数据）
    cv::Mat layer_mat =
        cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
                p_3d_grid_->get_data_ptr() + layer_offset);
    // 使用fillPoly填充车辆轮廓（值为100，表示障碍物占据）
    cv::fillPoly(layer_mat, vv_coord_cv, 100);
  }
  return kSuccess;
}

}  // namespace planning
