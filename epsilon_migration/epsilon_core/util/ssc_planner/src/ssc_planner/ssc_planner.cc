/**
 * @file ssc_planner.cc
 * @author GW
 * @brief SSC规划器核心实现：基于时空语义走廊（SSC）的轨迹生成框架
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 * 
 * **文件概述**：
 * 本文件实现了SSC（Spatio-temporal Semantic Corridor）规划器的核心功能，
 * 对应论文《基于时空语义走廊的复杂城市环境安全轨迹生成》中的算法实现。
 * 
 * **核心功能**（对应文档章节）：
 * 1. **SSC地图和走廊构建**（文档第5节：时空语义走廊）
 *    - 在Frenét坐标系（slt三维空间）中构建时空语义立方体地图
 *    - 实现算法1：语义走廊生成（种子生成、立方体膨胀、约束关联、立方体松弛）
 *    - 统一表示类障碍物语义元素（静态/动态障碍物、交通信号灯）和类约束语义元素（限速、换道约束）
 * 
 * 2. **轨迹优化**（文档第6节：具有安全性和可行性保证的轨迹生成）
 *    - 使用分段Bézier曲线参数化轨迹（公式1和2）
 *    - 最小化加加速度代价函数（公式3）
 *    - 施加边界约束（公式7）、自由空间约束（公式6，k=0）、
 *      动力学约束（公式6，k=1,2）和连续性约束（公式8）
 *    - 将优化问题转化为二次规划（QP）问题求解
 * 
 * 3. **坐标系转换**（文档第5.1节：语义元素与Frenét坐标系表示）
 *    - 将全局坐标系数据转换为Frenét坐标系
 *    - 利用Frenét坐标系的优势：语义元素与车道几何自然关联
 * 
 * **关键设计**：
 * - **Frenét坐标系**：基于参考车道的动态参考系，纵向s、横向l、时间t
 * - **分段Bézier曲线**：利用凸包和速端曲线特性，保证轨迹安全性和可行性
 * - **QP优化**：高效求解，满足实时性要求（20 Hz）
 * - **理论安全保证**：通过Bézier曲线的凸包特性，为整个轨迹提供理论安全保证
 * 
 * **主要函数**：
 * - Init()：初始化规划器，配置SSC地图参数
 * - RunOnce()：执行一次完整的规划循环（数据准备、状态转换、SSC构建、轨迹优化）
 * - RunQpOptimization()：执行QP优化生成Bézier轨迹
 * - StateTransformForInputData()：将输入数据转换为Frenét坐标系
 * 
 * **参考文档**：paper/SSC.md
 */
#include "ssc_planner/ssc_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>

/**
 * **OpenMP多线程支持配置**
 * 
 * **USE_OPENMP标志**：
 * - 0：禁用OpenMP，使用单线程坐标转换（默认）
 * - 1：启用OpenMP，使用多线程并行坐标转换
 * 
 * **性能考虑**：
 * - 单线程模式：简单可靠，适合中等规模数据转换
 * - 多线程模式：适合大规模数据转换，但性能可能受CPU调度影响
 * 
 * **使用位置**：
 * - StateTransformForInputData()：根据此标志选择单线程或多线程转换
 * - StateTransformUsingOpenMp()：多线程转换实现（需要USE_OPENMP=1）
 * - StateTransformSingleThread()：单线程转换实现（默认）
 * 
 * **注意**：性能可能受多核任务调度显著影响，建议根据实际环境测试选择
 */
// ! 注意：性能可能受多核任务调度显著影响
#define USE_OPENMP 0

namespace planning {

/**
 * @brief 获取规划器名称
 * @return 返回规划器名称字符串 "ssc_planner"
 * 
 * **功能**：
 * 返回规划器名称，用于系统识别和日志记录。
 * 
 * **命名规则**：
 * - 名称："ssc_planner"
 * - 对应：Spatio-temporal Semantic Corridor Planner
 * - 用于：日志输出、调试信息、系统接口标识
 */
std::string SscPlanner::Name() { return std::string("ssc_planner"); }

/**
 * @brief 初始化SSC规划器（对应文档系统概述）
 * @param config_path 配置文件路径
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **初始化流程**（对应文档第3节：系统概述）：
 * 1. 读取配置文件：加载规划器参数和地图配置
 * 2. 配置SSC地图：设置地图尺寸、分辨率和动力学边界
 * 3. 创建SSC地图对象：初始化时空语义立方体地图
 * 
 * **关键配置参数**：
 * 
 * **规划器参数**（用于轨迹优化）：
 * - weight_proximity：相似度代价权重（对应文档公式3中的w_f^σ）
 *   - 控制生成轨迹与参考状态的相似度
 *   - 用于公式3的相似度代价项：w_f^σ · Σ(f_j^σ(t_k) - r_jk^σ)²
 * 
 * - low_speed_threshold：低速阈值（对应文档第5.1节：横向独立性判断）
 *   - 当速度 > 阈值时，横向和纵向可独立规划（is_lateral_independent_ = true）
 *   - 当速度 ≤ 阈值时，使用Primitive轨迹连接方法（is_lateral_independent_ = false）
 * 
 * - velocity_singularity_eps：速度奇异性阈值
 *   - 防止Frenet坐标系在低速时的数值奇异性
 *   - 用于边界约束中的速度最小值设置
 * 
 * **SSC地图配置**（对应文档第5节：时空语义走廊）：
 * 
 * 1. **地图尺寸和分辨率**（slt三维配置空间）：
 *    - map_size[0,1,2]：纵向s、横向l、时间t三个维度的地图尺寸
 *    - map_resolution[0,1,2]：三个维度的地图分辨率（网格大小）
 *    - s_back_len：纵向坐标向后延伸长度（处理历史状态）
 * 
 * 2. **动力学边界**（对应文档公式6：动力学约束）：
 *    - kMaxLongitudinalVel/Min：纵向速度上下界（β_{j,+/-}^{s,(1)}）
 *    - kMaxLongitudinalAcc/Decel：纵向加速度上下界（β_{j,+/-}^{s,(2)}）
 *    - kMaxLateralVel：横向速度上界（β_{j,+}^{l,(1)}）
 *    - kMaxLateralAcc：横向加速度上界（β_{j,+}^{l,(2)}）
 *    - **物理意义**：用于公式6的充分条件约束，保证轨迹的动力学可行性
 *      β_{j,-}^σ ≤ d^k f_j^σ(t)/dt^k ≤ β_{j,+}^σ（k=1,2表示速度和加速度）
 * 
 * 3. **时间维度配置**：
 *    - kMaxNumOfGridAlongTime：沿时间轴的最大网格数
 *      - 限制规划时域的时间分辨率
 *      - 对应文档算法1中的时间维度处理
 * 
 * 4. **障碍物膨胀**（可选，当前已禁用）：
 *    - inflate_steps[0-5]：六个方向的障碍物膨胀步数
 *    - 用于增加安全裕量，但会增加计算时间
 *    - 当前EPSILON项目中为提升效率已禁用
 * 
 * **SSC地图对象创建**：
 * - 创建SscMap对象，用于后续的SSC地图构建和走廊生成
 * - 地图对象将用于实现文档第5.2节的算法1（语义走廊生成）
 */
ErrorType SscPlanner::Init(const std::string config_path) {
  // 读取配置文件（解析protobuf格式的配置）
  ReadConfig(config_path);

  // * 规划器配置信息输出
  printf("\nSscPlanner Config:\n");
  printf(" -- weight_proximity: %lf\n", cfg_.planner_cfg().weight_proximity());

  LOG(INFO) << "[Ssc]SscPlanner Config:";
  LOG(INFO) << "[Ssc] -- low spd threshold: "
            << cfg_.planner_cfg().low_speed_threshold();
  LOG(INFO) << "[Ssc] -- weight_proximity: "
            << cfg_.planner_cfg().weight_proximity();

  /**
   * **SSC地图配置设置（对应文档第5节：时空语义走廊）**
   * 
   * 配置SSC地图的尺寸、分辨率和动力学边界参数，
   * 这些参数定义了slt三维配置空间的结构和约束范围。
   */
  // * SSC地图配置：设置地图的尺寸、分辨率和动力学边界
  SscMap::Config map_cfg;
  
  // 地图尺寸（slt三个维度：纵向s、横向l、时间t）
  map_cfg.map_size[0] = cfg_.map_cfg().map_size_x();  // 纵向s维度
  map_cfg.map_size[1] = cfg_.map_cfg().map_size_y();  // 横向l维度
  map_cfg.map_size[2] = cfg_.map_cfg().map_size_z();  // 时间t维度
  
  // 地图分辨率（slt三个维度：网格大小）
  map_cfg.map_resolution[0] = cfg_.map_cfg().map_resl_x();  // 纵向分辨率
  map_cfg.map_resolution[1] = cfg_.map_cfg().map_resl_y();  // 横向分辨率
  map_cfg.map_resolution[2] = cfg_.map_cfg().map_resl_z();  // 时间分辨率
  
  // 纵向坐标向后延伸长度（处理历史状态）
  map_cfg.s_back_len = cfg_.map_cfg().s_back_len();
  
  /**
   * **动力学边界配置（对应文档公式6：动力学约束）**
   * 
   * 设置速度和加速度的上下界，用于公式6的充分条件约束：
   * β_{j,-}^σ ≤ d^k f_j^σ(t)/dt^k ≤ β_{j,+}^σ
   * 
   * - k=1：速度约束（β^{σ,(1)}）
   * - k=2：加速度约束（β^{σ,(2)}）
   */
  // 纵向速度最大值：β_{j,+}^{s,(1)}（公式6，k=1，σ=s）
  map_cfg.kMaxLongitudinalVel = cfg_.map_cfg().dyn_bounds().max_lon_vel();
  
  // 纵向速度最小值：β_{j,-}^{s,(1)}（公式6，k=1，σ=s）
  // 注意：取配置值和速度奇异性阈值中的较大者，防止Frenet坐标系奇异性
  map_cfg.kMinLongitudinalVel =
      std::max(cfg_.map_cfg().dyn_bounds().min_lon_vel(),
               cfg_.planner_cfg().velocity_singularity_eps());
  
  // 纵向加速度最大值：β_{j,+}^{s,(2)}（公式6，k=2，σ=s）
  map_cfg.kMaxLongitudinalAcc = cfg_.map_cfg().dyn_bounds().max_lon_acc();
  
  // 纵向减速度最大值：β_{j,-}^{s,(2)}（公式6，k=2，σ=s，负值）
  map_cfg.kMaxLongitudinalDecel = cfg_.map_cfg().dyn_bounds().max_lon_dec();
  
  // 横向速度最大值：β_{j,+}^{l,(1)}（公式6，k=1，σ=l）
  map_cfg.kMaxLateralVel = cfg_.map_cfg().dyn_bounds().max_lat_vel();
  
  // 横向加速度最大值：β_{j,+}^{l,(2)}（公式6，k=2，σ=l）
  map_cfg.kMaxLateralAcc = cfg_.map_cfg().dyn_bounds().max_lat_acc();
  
  // 沿时间轴的最大网格数（限制规划时域的时间分辨率）
  map_cfg.kMaxNumOfGridAlongTime = cfg_.map_cfg().max_grids_along_time();
  
  /**
   * **障碍物膨胀配置（当前已禁用）**
   * 
   * 障碍物膨胀可在障碍物周围增加安全裕量，但会增加计算时间。
   * 当前EPSILON项目中为提升效率已禁用此功能。
   * 
   * inflate_steps[0-5]：六个方向的膨胀步数
   * - [0]：纵向正方向（s+）
   * - [1]：纵向负方向（s-）
   * - [2]：横向正方向（l+）
   * - [3]：横向负方向（l-）
   * - [4]：时间正方向（t+）
   * - [5]：时间负方向（t-）
   */
  map_cfg.inflate_steps[0] = cfg_.map_cfg().infl_steps().x_p();
  map_cfg.inflate_steps[1] = cfg_.map_cfg().infl_steps().x_n();
  map_cfg.inflate_steps[2] = cfg_.map_cfg().infl_steps().y_p();
  map_cfg.inflate_steps[3] = cfg_.map_cfg().infl_steps().y_n();
  map_cfg.inflate_steps[4] = cfg_.map_cfg().infl_steps().z_p();
  map_cfg.inflate_steps[5] = cfg_.map_cfg().infl_steps().z_n();
  
  /**
   * **创建SSC地图对象**
   * 
   * 创建SscMap对象，用于后续的：
   * 1. SSC地图构建（ConstructSscMap）- 文档第5.1节：类障碍物语义元素
   * 2. 走廊生成（ConstructCorridorUsingInitialTrajectory）- 文档算法1
   * 3. 立方体提取（GetFinalGlobalMetricCubesList）- 用于轨迹优化
   */
  p_ssc_map_ = new SscMap(map_cfg);

  return kSuccess;
}

/**
 * @brief 从配置文件读取规划器参数
 * @param config_path 配置文件路径（protobuf文本格式）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 使用protobuf文本格式解析配置文件，将配置内容加载到cfg_成员变量中。
 * 
 * **配置文件内容**（对应文档第3节：系统概述）：
 * 
 * **规划器参数**（planner_cfg）：
 * - weight_proximity：相似度代价权重（用于轨迹优化）
 * - low_speed_threshold：低速阈值（判断横向独立性）
 * - velocity_singularity_eps：速度奇异性阈值（防止Frenet坐标系奇异）
 * - is_fitting_only：是否仅拟合模式（跳过SSC地图构建）
 * 
 * **地图配置参数**（map_cfg）：
 * - map_size_x/y/z：slt三个维度的地图尺寸
 * - map_resl_x/y/z：slt三个维度的地图分辨率
 * - s_back_len：纵向坐标向后延伸长度
 * - dyn_bounds：动力学边界（速度、加速度的上下界）
 * - infl_steps：障碍物膨胀步数（六个方向）
 * - max_grids_along_time：沿时间轴的最大网格数
 * 
 * **文件格式**：
 * - 使用protobuf文本格式（.proto定义）
 * - 通过TextFormat::Parse解析
 * - 解析失败时会终止程序（assert）
 * 
 * **对应文档**：第3节（系统概述）、第5节（SSC地图配置）
 */
ErrorType SscPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading ssc planner config\n");
  using namespace google::protobuf;
  
  // 打开配置文件（只读模式）
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  
  // 使用protobuf的TextFormat解析配置文件内容
  // 将文本格式的配置转换为cfg_对象（protobuf消息）
  TextFormat::Parse(&fstream, &cfg_);
  
  // 检查配置是否初始化成功（protobuf验证）
  // 如果配置解析失败或必填字段缺失，会终止程序
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

/**
 * @brief 设置初始状态（用于规划起始）
 * @param state 车辆的初始状态（全局坐标系）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 设置规划器的初始状态，并标记已设置初始状态标志。
 * 该初始状态将作为轨迹优化的起始边界条件（对应文档公式7）。
 * 
 * **初始状态内容**（State结构）：
 * - 位置：vec_position（全局坐标 x, y）
 * - 速度：velocity（速度大小）
 * - 加速度：acceleration（加速度大小）
 * - 姿态：angle（车辆朝向角）
 * - 角速度：curvature（曲率）
 * - 时间戳：time_stamp（初始时间）
 * 
 * **用途**（对应文档公式7：期望状态约束）：
 * - 设置轨迹优化的起始边界条件
 *   - 位置约束：f^σ(t_0) = σ_{t_0}^{(0)}（公式7，k=0）
 *   - 速度约束：df^σ(t_0)/dt = σ_{t_0}^{(1)}（公式7，k=1）
 *   - 加速度约束：d²f^σ(t_0)/dt² = σ_{t_0}^{(2)}（公式7，k=2）
 * - 在RunOnce()中，如果没有预设初始状态，则使用当前自车状态
 * 
 * **标志作用**：
 * - has_initial_state_：标记是否已设置初始状态
 * - 如果未设置，RunOnce()会使用ego_vehicle_.state()作为初始状态
 * 
 * **对应文档**：第6.3节（实施安全性和动力学约束）、公式7（期望状态约束）
 */
ErrorType SscPlanner::set_initial_state(const State& state) {
  // 保存初始状态（用于后续的轨迹优化）
  initial_state_ = state;
  // 标记已设置初始状态（RunOnce()中会检查此标志）
  has_initial_state_ = true;
  return kSuccess;
}

/**
 * @brief 执行一次规划循环
 * @return 返回错误类型，kSuccess表示成功
 * 
 * 这是SSC规划器的核心函数，执行一次完整的规划流程：
 * 1. 数据准备：获取自车信息、参考车道、障碍物信息等
 * 2. 状态转换：将全局坐标系数据转换为Frenet坐标系
 * 3. SSC地图构建：构建时空语义立方体地图和走廊
 * 4. 轨迹优化：执行QP优化生成平滑轨迹
 * 5. 轨迹选择：根据当前行为选择最终轨迹
 */
ErrorType SscPlanner::RunOnce() {
  /**
   * **规划循环初始化**
   * 
   * 记录当前规划周期的时间戳和性能计时。
   * 
   * **时间戳**：
   * - stamp_：当前规划周期的时间戳
   * - 用于日志记录和性能分析
   * 
   * **性能计时**：
   * - ssc_timer：总计时器（整个规划循环的耗时）
   * - timer_prepare：数据准备阶段计时
   * - timer_stf：状态转换阶段计时
   * - timer_sscmap：SSC地图构建阶段计时
   * - timer_opt：轨迹优化阶段计时
   * 
   * **性能目标**（对应文档第7节：实验结果）：
   * - 规划频率：20 Hz（50ms/周期）
   * - 各阶段耗时监控用于性能优化
   */
  // 获取当前时间戳（用于日志记录和性能分析）
  stamp_ = map_itf_->GetTimeStamp();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE START: " << stamp_
               << " ********************\n";
  // 启动总计时器（记录整个规划循环的耗时，目标：< 50ms，即20Hz）
  static TicToc ssc_timer;
  ssc_timer.tic();

  // ========== 阶段1: 数据准备 ==========
  static TicToc timer_prepare;
  timer_prepare.tic();
  // 获取自车信息
  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego vehicle info.";
    return kWrongStatus;
  }

  // 设置规划起始状态：如果没有预设初始状态，则使用当前自车状态
  if (!has_initial_state_) {
    initial_state_ = ego_vehicle_.state();
  }
  has_initial_state_ = false;

  /**
   * **横向独立性判断（对应文档第5.1节：Frenét坐标系优势）**
   * 
   * 当速度大于低速阈值时，横向和纵向运动可独立规划。
   * 
   * **物理意义**：
   * - **高速情况**（is_lateral_independent_ = true）：
   *   车辆运动可分解为独立的纵向 $s$ 和横向 $l$ 运动
   *   这是Frenet坐标系的核心优势（文档第5.1节）
   * - **低速情况**（is_lateral_independent_ = false）：
   *   车辆运动学模型接近奇异点，横向和纵向耦合
   *   需要使用Primitive轨迹连接方法（非Bézier优化）
   * 
   * **影响**：
   * - 高速：使用Bézier曲线优化（公式2），独立优化纵向和横向
   * - 低速：使用Primitive轨迹连接，避免Frenet坐标系奇异性问题
   */
  // 判断是否横向独立：当速度大于低速阈值时，横向和纵向可独立规划
  is_lateral_independent_ =
      initial_state_.velocity > cfg_.planner_cfg().low_speed_threshold()
          ? true
          : false;
  
  /**
   * **参考车道获取（对应文档第5.1节：Frenét坐标系基准）**
   * 
   * 获取局部参考车道，这是构建Frenet坐标系的基础。
   * 
   * **Frenet坐标系定义**（文档第5.1节）：
   * - **纵向 $s$**：沿参考车道的弧长距离（从起点开始累积）
   * - **横向 $l$**：垂直于参考车道方向的偏移距离
   * - 参考车道通常从路径规划器提供的路径信息中提取
   */
  // 获取局部参考车道（用于Frenet坐标系转换）
  if (map_itf_->GetLocalReferenceLane(&nav_lane_local_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to find ego lane.";
    return kWrongStatus;
  }
  
  /**
   * **状态转换器初始化**
   * 
   * 创建StateTransformer对象，用于全局坐标系和Frenet坐标系之间的转换。
   * 转换器基于参考车道 nav_lane_local_ 构建Frenet坐标系。
   */
  // 创建状态转换器（用于全局坐标和Frenet坐标之间的转换）
  stf_ = common::StateTransformer(nav_lane_local_);

  /**
   * **初始状态Frenet坐标转换**
   * 
   * 将自车的初始状态从全局坐标系转换为Frenet坐标系。
   * 
   * **用途**：
   * 1. 初始化SSC地图（ResetSscMap）
   * 2. 设置轨迹优化的起始边界约束（文档公式7）
   *    - 位置：initial_frenet_state_.vec_s[0], vec_dt[0]
   *    - 速度：initial_frenet_state_.vec_s[1], vec_dt[1]
   *    - 加速度：initial_frenet_state_.vec_s[2], vec_dt[2]
   */
  // 将初始状态转换为Frenet坐标系
  if (stf_.GetFrenetStateFromState(initial_state_, &initial_frenet_state_) !=
      kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get init state frenet state.";
    return kWrongStatus;
  }

  /**
   * **获取自车的离散行为（对应文档第3节：行为规划）**
   * 
   * 从行为规划器（如MPDM）获取当前自车的离散行为决策。
   * 
   * **行为类型**：
   * - kLaneKeeping：车道保持
   * - kLaneChangeLeft：左变道
   * - kLaneChangeRight：右变道
   * 
   * **用途**：
   * - 用于选择对应的SSC走廊和优化轨迹（UpdateTrajectoryWithCurrentBehavior）
   * - 对应文档第3节：行为规划层的输出作为运动规划的输入
   */
  // 获取自车的离散行为（如车道保持、左变道、右变道等）
  if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego behavior.";
    return kWrongStatus;
  }

  /**
   * **获取障碍物地图（对应文档第5.1节：类障碍物语义元素）**
   * 
   * 获取静态障碍物的占据栅格地图，用于SSC地图构建。
   * 
   * **用途**：
   * - 用于构建SSC地图中的静态障碍物（文档第5.1节）
   * - 静态障碍物可视为跨越整个时间轴的障碍物
   */
  // 获取障碍物地图
  if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle map.";
    return kWrongStatus;
  }

  /**
   * **获取障碍物网格点（对应文档第5.1节：类障碍物语义元素）**
   * 
   * 获取静态障碍物的占据网格点（离散点表示），用于SSC地图构建。
   * 
   * **用途**：
   * - 与障碍物地图一起，用于构建SSC地图中的静态障碍物
   * - 在Frenet坐标系中表示障碍物的占据位置
   */
  // 获取障碍物网格点
  if (map_itf_->GetObstacleGrids(&obstacle_grids_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle grids.";
    return kWrongStatus;
  }

  /**
   * **获取前向轨迹（对应文档第3节：前向仿真状态）**
   * 
   * 从行为规划器获取前向仿真轨迹，包括：
   * 1. **自车的前向仿真轨迹**：每个行为的候选轨迹（用于SSC走廊生成的种子）
   * 2. **周围车辆的前向轨迹**：其他车辆的预测轨迹（用于SSC地图构建）
   * 
   * **用途**（对应文档算法1：种子生成）：
   * - forward_trajs_：作为SSC走廊生成的种子（算法1第3行）
   * - surround_forward_trajs_：用于构建SSC地图中的动态障碍物（文档第5.1节）
   * 
   * **对应文档**：第3节（系统概述）、第5.2节（算法1）、第5.1节（动态障碍物）
   */
  // 获取前向轨迹（包括自车的前向仿真轨迹和周围车辆的前向轨迹）
  if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                       &surround_forward_trajs_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get forward trajectories.";
    return kWrongStatus;
  }

  // 记录数据准备阶段耗时
  auto t_prepare = timer_prepare.toc();
  LOG(WARNING) << "[Ssc]prepare time cost: " << t_prepare << " ms";

  /**
   * **阶段2: 状态转换（对应文档第5.1节：语义元素与Frenét坐标系表示）**
   * 
   * 将所有输入数据从全局坐标系（Cartesian坐标系）转换为Frenet坐标系。
   * 
   * **为什么使用Frenet坐标系**（文档第5.1节）：
   * 1. **语义元素关联**：大多数语义元素（限速、交通信号灯、停车标志）都与车道几何相关联
   *    - 限速通常与车道的特定纵向范围相关
   *    - 交通信号灯和停车标志通常位于特定纵向位置
   * 2. **类人驾驶行为**：驾驶行为可分解为横向运动和纵向运动
   *    - 纵向 $s$：沿参考车道的弧长距离
   *    - 横向 $l$：垂直于参考车道方向的偏移距离
   *    - 在 $s$ 和 $l$ 方向对自由空间建模比在笛卡尔坐标系中更直观
   * 3. **时间维度**：添加时间 $t$ 形成 slt 三维配置空间
   *    - 动态障碍物可视为时间域中的一系列静态障碍物
   *    - 预测轨迹是时间剖面的，可视为一系列时空障碍物
   * 
   * **转换内容**：
   * 1. 自车状态：初始状态和当前状态
   * 2. 自车前向仿真轨迹：每个行为的前向仿真轨迹
   * 3. 周围车辆轨迹：周围车辆的前向轨迹（用于SSC地图构建）
   * 4. 障碍物网格点：静态障碍物的占据网格点
   * 
   * **Frenet坐标系定义**（文档第5.1节）：
   * - **纵向 $s$**：沿参考车道的弧长距离（从起点开始累积）
   * - **横向 $l$**：垂直于参考车道方向的偏移距离（左正右负或左负右正）
   * - **时间 $t$**：时间维度，用于处理动态障碍物和时变约束
   */
  // ========== 阶段2: 状态转换 ==========
  static TicToc timer_stf;
  timer_stf.tic();
  if (StateTransformForInputData() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to transform state into ff.";
    return kWrongStatus;
  }
  auto t_stf = timer_stf.toc();
  LOG(WARNING) << "[Ssc]state transform time cost: " << t_stf << " ms";

  // ========== 阶段3: SSC地图和走廊构建 ==========
  // 注意：SSC地图构建部分有时可能非常慢（可能与CPU调度有关）
  static TicToc timer_sscmap;
  timer_sscmap.tic();
  
  /**
   * **SSC地图和走廊构建（对应文档第5节：时空语义走廊）**
   * 
   * 本阶段实现文档第5.2节"语义走廊生成"算法（算法1），包括：
   * 1. **种子生成**：从前向仿真轨迹中提取种子状态
   * 2. **立方体膨胀**：以种子为中心，在slt三维空间中膨胀生成初始立方体
   * 3. **约束关联**：将语义边界（限速、交通规则等）与立方体关联
   * 4. **立方体松弛**：在满足硬约束的前提下，松弛立方体边界以增加优化空间
   * 
   * **对应文档**：
   * - 第5.1节：语义元素与Frenét坐标系表示
   * - 第5.2节：语义走廊生成算法（算法1）
   * - 第5.3节：立方体膨胀、约束关联、立方体松弛的具体实现
   */
  
  // 设置时间原点（用于将全局时间转换为相对时间）
  time_origin_ = initial_state_.time_stamp;
  
  // 重置SSC地图，使用初始Frenet状态初始化
  // 初始化地图的坐标系、分辨率、动力学边界等参数
  p_ssc_map_->ResetSscMap(initial_frenet_state_);
  
  // 为每个行为构建SSC地图和走廊（用于闭环仿真预测）
  // 每个行为（如车道保持、左变道、右变道）对应一个独立的走廊
  int num_behaviors = forward_behaviors_.size();
  for (int i = 0; i < num_behaviors; ++i) {
    // 如果不是仅拟合模式，需要构建SSC地图（包含障碍物信息）
    if (!cfg_.planner_cfg().is_fitting_only()) {
      /**
       * **SSC地图构建（对应文档第5.1节：类障碍物语义元素）**
       * 
       * ConstructSscMap() 实现的功能：
       * 1. 将静态障碍物渲染为跨越整个时间轴的障碍物
       * 2. 将动态障碍物（其他车辆的预测轨迹）渲染为时间域中的一系列静态障碍物
       * 3. 将交通信号灯（红灯）渲染为占据特定纵向位置和时间段的障碍物
       * 4. 生成三维占据网格（slt域）
       * 
       * **输入**：
       * - surround_forward_trajs_fs_[i]：第i个行为对应的周围车辆前向轨迹（Frenet坐标系）
       * - obstacle_grids_fs_：障碍物网格点（Frenet坐标系）
       */
      if (p_ssc_map_->ConstructSscMap(surround_forward_trajs_fs_[i],
                                      obstacle_grids_fs_)) {
        LOG(ERROR) << "[Ssc]fail to construct ssc map.";
        return kWrongStatus;
      }
    }
    
    // 注意：为节省时间，当前未执行障碍物膨胀（inflate）
    // 障碍物膨胀会在障碍物周围增加安全裕量，但会增加计算时间
    // 在EPSILON项目中为提升效率已禁用，未来可优化
    // ! Notice: No inflation here to save time in eudm project. Improve
    // ! efficiency in the future.
    // TicToc timer_infl;
    // p_ssc_map_->InflateObstacleGrid(ego_vehicle_.param());
    // printf("[SscPlanner] InflateObstacleGrid time cost: %lf ms\n",
    //        timer_infl.toc());
    
    /**
     * **SSC走廊构建（对应文档第5.2节：语义走廊生成算法）**
     * 
     * ConstructCorridorUsingInitialTrajectory() 实现算法1的核心流程：
     * 
     * 1. **种子生成**（算法1第3行）：
     *    - 从前向仿真轨迹 forward_trajs_fs_[i] 中提取离散种子状态
     *    - 当前实现会用相邻两个种子构造初始立方体，再执行膨胀
     * 
     * 2. **立方体膨胀**（算法1第4行）：
     *    - 以种子为中心，在slt三个方向交替膨胀
     *    - 遇到障碍物或语义边界时终止膨胀
     *    - 处理语义边界的特殊逻辑（禁用与进入方向相反的膨胀方向）
     * 
     * 3. **约束关联**（算法1第5行）：
     *    - 将语义边界（限速、交通规则等）与立方体关联
     *    - 区分硬约束（如限速）和软约束（如换道持续时间）
     * 
     * 4. **立方体松弛**（算法1第6行）：
     *    - 在满足硬约束的前提下，松弛立方体边界
     *    - 为优化过程预留额外空间
     *    - 根据速度约束和换道持续时间计算允许的松弛裕量
     * 
     * **输入**：
     * - p_3d_grid()：三维占据网格（包含障碍物信息）
     * - forward_trajs_fs_[i]：第i个行为的前向仿真轨迹（作为种子）
     * 
     * **输出**：
     * - 每个行为对应一个SSC走廊（时空语义立方体序列）
     * - 每个立方体定义了一个时空自由空间区域
     */
    if (p_ssc_map_->ConstructCorridorUsingInitialTrajectory(
            p_ssc_map_->p_3d_grid(), forward_trajs_fs_[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail to construct corridor for behavior " << i;
      return kWrongStatus;
    }
  }
  
  /**
   * **获取最终度量化立方体列表（用于轨迹优化）**
   * 
   * GetFinalGlobalMetricCubesList() 实现的功能：
   * - 将三维网格索引立方体转换为连续度量值边界
   * - 提取最终用于轨迹优化的立方体序列
   * 
   * **转换原因**：
   * - 走廊生成阶段使用的是离散网格坐标
   * - QP优化需要连续的 s/l/t 边界，而不是网格索引
   * - 这里仍然是 Frenet 纵向/横向与时间的度量空间，不是笛卡尔全局坐标
   * 
   * **输出**：
   * - final_corridor_vec()：每个行为对应的最终立方体列表
   * - if_corridor_valid()：每个走廊的有效性标志
   * 
   * **对应文档**：第5.2节（语义走廊生成）、第6节（轨迹优化）
   */
  // 获取最终度量化立方体列表（用于轨迹优化）
  // 将离散网格索引恢复为连续的 s/l/t 约束边界
  if (kSuccess != p_ssc_map_->GetFinalGlobalMetricCubesList()) {
    LOG(ERROR) << "[Ssc]fail to get final corridor";
    return kWrongStatus;
  }
  auto t_sscmap = timer_sscmap.toc();
  LOG(WARNING) << "[Ssc]construct ssc map and corridor time cost: " << t_sscmap
               << " ms";

  /**
   * **阶段4: QP优化（对应文档第6节：具有安全性和可行性保证的轨迹生成）**
   * 
   * RunQpOptimization() 实现的功能：
   * 1. 为每个行为生成分段Bézier曲线（公式1和2）
   * 2. 优化代价函数，最小化加加速度（公式3）
   * 3. 施加边界约束（公式7）、自由空间约束（公式6，k=0）、
   *    动力学约束（公式6，k=1,2）和连续性约束（公式8）
   * 4. 将优化问题转化为二次规划（QP）问题求解
   * 
   * **输出**：
   * - qp_trajs_：优化后的Bézier样条轨迹列表（每个行为对应一个）
   * - valid_behaviors_：有效行为列表
   * - corridors_：对应的SSC走廊列表
   */
  static TicToc timer_opt;
  timer_opt.tic();
  if (RunQpOptimization() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to optimize qp trajectories.\n";
    return kWrongStatus;
  }

  /**
   * **轨迹选择（对应文档第3节：轨迹生成框架）**
   * 
   * UpdateTrajectoryWithCurrentBehavior() 实现的功能：
   * - 从多个候选轨迹中选择当前行为对应的轨迹
   * - 如果当前行为的轨迹不可用，选择车道保持（kLaneKeeping）作为备选
   * 
   * **选择策略**：
   * 1. **精确匹配**：优先选择与当前行为（ego_behavior_）完全匹配的轨迹
   *    - 如果行为规划器输出"左变道"，选择左变道对应的优化轨迹
   * 2. **备选策略**：如果精确匹配失败，选择车道保持（kLaneKeeping）轨迹
   *    - 车道保持是最安全的备选行为，通常总是可行
   * 
   * **输出轨迹**：
   * - trajectory_：主要轨迹（Frenet坐标系中的Bézier样条）
   * - low_spd_alternative_traj_：低速备选轨迹（Primitive轨迹）
   * - final_corridor_：最终选择的SSC走廊
   * - final_ref_states_：最终选择的参考状态列表
   * 
   * **对应文档**：第3节（系统概述）、第6节（轨迹生成）
   */
  if (UpdateTrajectoryWithCurrentBehavior() != kSuccess) {
    // 轨迹选择失败时的错误日志（包含调试信息）
    LOG(ERROR) << "[Ssc]fail: current behavior "
               << static_cast<int>(ego_behavior_) << " not valid.";
    LOG(ERROR) << "[Ssc]fail: has " << qp_trajs_.size() << " traj, "
               << valid_behaviors_.size() << " behaviors.";
    return kWrongStatus;
  }

  /**
   * **轨迹验证（可选，当前已禁用）**
   * 
   * 如果启用，会验证生成轨迹的有效性（边界约束、曲率约束等）。
   * 当前代码中此功能被禁用（#if 0），用于调试和验证。
   * 
   * **验证内容**（ValidateTrajectory函数）：
   * - 起始状态验证（对应文档公式7）
   * - 终止状态验证
   * - 动力学可行性验证（曲率约束）
   */
#if 0
  auto traj = trajectory();
  if (ValidateTrajectory(*traj) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: infeasible traj.";
    return kWrongStatus;
  }
#endif

  /**
   * **性能统计和日志记录（对应文档第7节：实验结果）**
   * 
   * 记录各阶段的耗时，用于性能分析和优化。
   * 
   * **性能指标**（对应文档第7.1节）：
   * - 数据准备耗时：t_prepare
   * - 状态转换耗时：t_stf
   * - SSC地图构建耗时：t_sscmap
   * - 轨迹优化耗时：t_opt
   * - 总耗时：time_cost_（目标：< 50ms，即20Hz）
   * 
   * **性能目标**（对应文档第7.1节：实现细节）：
   * - 规划频率：20 Hz（50ms/周期）
   * - 实际测试：可稳定运行于20 Hz
   * 
   * **日志输出**：
   * - 各阶段耗时：用于定位性能瓶颈
   * - 时间总和与总计时器的差值：用于检测计时误差
   * - 规划周期标记：RUNONCE START/FINISH，用于日志追踪
   */
  // 记录轨迹优化阶段耗时
  auto t_opt = timer_opt.toc();
  LOG(WARNING) << "[Ssc]optimization time cost: " << t_opt << " ms";

  // 计算各阶段时间总和（用于验证计时准确性）
  auto t_sum = t_prepare + t_stf + t_sscmap + t_opt;
  // 获取总计时器的实际耗时
  time_cost_ = ssc_timer.toc();
  // 输出时间总和与总计时器的差值（用于检测计时误差）
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]Sum of time: " << t_sum
               << " ms, diff: " << time_cost_ - t_sum << " ms";
  // 输出规划周期结束标记和总耗时（目标：< 50ms，即20Hz）
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE FINISH: " << stamp_ << " +"
               << time_cost_ << " ms ********************\n";

  return kSuccess;
}

/**
 * @brief 执行QP优化生成Bézier轨迹
 * @return 返回错误类型，kSuccess表示成功
 * 
 * 本函数实现了SSC.md文档第6节"具有安全性和可行性保证的轨迹生成"中的QP优化方法。
 * 
 * **核心流程**：
 * 1. 获取SSC走廊（每个行为对应一个走廊）
 * 2. 为每个走廊设置边界约束（对应文档公式7）
 * 3. 生成分段Bézier曲线（对应文档公式1和2）
 * 4. 优化代价函数（对应文档公式3）
 * 5. 施加安全性和动力学约束（对应文档公式4-8）
 * 
 * **关键公式对应**：
 * - 公式(1)：Bézier曲线定义 f(t) = Σ p_i · b_m^i(t)
 * - 公式(2)：分段Bézier曲线表示（每段与一个SSC立方体对应）
 * - 公式(3)：代价函数 J_j = w_s ∫(d³f^s/dt³)²dt + w_l ∫(d³f^l/dt³)²dt
 * - 公式(4)：导数控制点递推关系
 * - 公式(5)：边界导数约束
 * - 公式(6)：充分条件约束（安全性和动力学约束）
 * - 公式(7)：期望状态约束（边界条件）
 * - 公式(8)：连续性约束（当前实现对相邻段施加到加速度，即 C² 连续）
 */
ErrorType SscPlanner::RunQpOptimization() {
  // 获取所有行为的最终走廊列表（每个走廊对应一个行为策略）
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> cube_list =
      p_ssc_map_->final_corridor_vec();
  // 获取每个走廊的有效性标志
  std::vector<int> if_corridor_valid = p_ssc_map_->if_corridor_valid();
  if (cube_list.empty()) return kWrongStatus;
  if (cube_list.size() != forward_behaviors_.size()) {
    LOG(ERROR) << "[Ssc]cube list " << static_cast<int>(cube_list.size())
               << " not consist with behavior size: "
               << static_cast<int>(forward_behaviors_.size())
               << ", forward traj " << static_cast<int>(forward_trajs_.size())
               << ", flag size " << static_cast<int>(if_corridor_valid.size());
    return kWrongStatus;
  }

  // 清空输出容器
  qp_trajs_.clear();
  primitive_trajs_.clear();
  valid_behaviors_.clear();
  corridors_.clear();
  ref_states_list_.clear();
  
  // 为每个行为（走廊）生成Bézier轨迹
  for (int i = 0; i < static_cast<int>(cube_list.size()); i++) {
    int beh = static_cast<int>(forward_behaviors_[i]);
    if (if_corridor_valid[i] == 0) {
      LOG(ERROR) << "[Ssc]fail: for behavior "
                 << static_cast<int>(forward_behaviors_[i])
                 << " has no valid corridor.";
      continue;
    }

    auto fs_vehicle_traj = forward_trajs_fs_[i];
    int num_states = static_cast<int>(fs_vehicle_traj.size());

    /**
     * **起始边界约束设置（对应文档公式7：期望状态约束）**
     * 
     * 公式(7)：\frac{d^k f_0^σ(t_0)}{dt^k} = σ_{t_0}^{(k)}
     * 
     * 起始约束向量包含三个元素，分别对应：
     * - start_constraints[0]：位置约束 (k=0)
     *   - 纵向位置：ego_frenet_state_.vec_s[0]（自车当前纵向位置 s）
     *   - 横向位置：ego_frenet_state_.vec_dt[0]（自车当前横向位置 l）
     * 
     * - start_constraints[1]：速度约束 (k=1)
     *   - 纵向速度：max(vec_s[1], velocity_singularity_eps)（防止速度奇异性）
     *   - 横向速度：vec_dt[1]（自车当前横向速度）
     *   - 注意：当速度接近0时会出现Frenet坐标系奇异性，需要设置最小阈值
     * 
     * - start_constraints[2]：加速度约束 (k=2)
     *   - 纵向加速度：vec_s[2]（自车当前纵向加速度）
     *   - 横向加速度：vec_dt[2]（自车当前横向加速度）
     * 
     * **物理意义**：确保生成的Bézier轨迹从当前自车状态平滑起始
     */
    vec_E<Vecf<2>> start_constraints;
    // 位置约束：f^σ(t_0) = σ_{t_0}^{(0)}（公式7，k=0）
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[0], ego_frenet_state_.vec_dt[0]));
    // 速度约束：df^σ(t_0)/dt = σ_{t_0}^{(1)}（公式7，k=1）
    // 使用velocity_singularity_eps防止Frenet坐标系在低速时的奇异性
    start_constraints.push_back(
        Vecf<2>(std::max(ego_frenet_state_.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                ego_frenet_state_.vec_dt[1]));
    // 加速度约束：d²f^σ(t_0)/dt² = σ_{t_0}^{(2)}（公式7，k=2）
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[2], ego_frenet_state_.vec_dt[2]));

    /**
     * **终止边界约束设置（对应文档公式7：期望状态约束）**
     * 
     * 公式(7)：\frac{d^k f_n^σ(t_n)}{dt^k} = σ_{t_n}^{(k)}
     * 
     * 终止约束向量包含两个元素（位置和速度），加速度约束可选：
     * - end_constraints[0]：位置约束 (k=0)
     *   - 目标纵向位置：前向仿真轨迹最后一个状态的纵向位置
     *   - 目标横向位置：前向仿真轨迹最后一个状态的横向位置
     * 
     * - end_constraints[1]：速度约束 (k=1)
     *   - 目标纵向速度：前向仿真轨迹最后一个状态的纵向速度
     *   - 目标横向速度：前向仿真轨迹最后一个状态的横向速度
     * 
     * 注意：终止加速度约束被注释掉，通常不强制要求终止加速度，以提高优化灵活性
     * 
     * **物理意义**：确保生成的Bézier轨迹平滑收敛到行为规划器提供的目标状态
     */
    vec_E<Vecf<2>> end_constraints;
    // 终止位置约束：f^σ(t_n) = σ_{t_n}^{(0)}（公式7，k=0）
    end_constraints.push_back(
        Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[0],
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[0]));
    // 终止速度约束：df^σ(t_n)/dt = σ_{t_n}^{(1)}（公式7，k=1）
    end_constraints.push_back(
        Vecf<2>(std::max(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[1]));
    // 终止加速度约束（可选，当前未使用）
    // end_constraints.push_back(
    //     Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[2],
    //             fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[2]));
    
    /**
     * **Bézier样条生成器初始化**
     * 
     * SplineGenerator<5, 2> 参数说明：
     * - 模板参数<5, 2>：5表示Bézier曲线的度数（degree-5），2表示二维（纵向s和横向l）
     * - degree-5 Bézier曲线：f(t) = Σ_{i=0}^5 p_i · b_5^i(t)（公式1，m=5）
     * - 优势：5次Bézier曲线可以表示三次导数（加加速度），满足平滑性要求
     * 
     * **对应文档**：
     * - 公式(1)：Bézier曲线定义 f(t) = Σ p_i · b_m^i(t)
     * - 公式(2)：分段Bézier曲线表示（每段对应一个SSC立方体）
     */
    common::SplineGenerator<5, 2> spline_generator;
    BezierSpline bezier_spline;

    // 设置走廊的时间上界为前向仿真轨迹的最后一个时间戳
    cube_list[i].back().t_ub = fs_vehicle_traj.back().frenet_state.time_stamp;

    // 检查走廊的可行性（确保相邻立方体时间连续，对应文档公式8的连续性要求）
    if (CorridorFeasibilityCheck(cube_list[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail: corridor not valid for optimization.";
      continue;
    }

    /**
     * **参考状态提取（用于相似度代价项）**
     * 
     * 从行为规划器提供的前向仿真轨迹中提取参考状态，用于：
     * 1. 提供轨迹优化的参考点（对应文档公式3中的相似度项）
     * 2. 确保生成的Bézier轨迹与行为规划决策一致
     * 
     * 注意：这些参考状态将作为代价函数中的相似度项，鼓励生成的轨迹接近行为规划结果
     */
    std::vector<decimal_t> ref_stamps;
    vec_E<Vecf<2>> ref_points;
    vec_E<common::FrenetState> ref_states;
    for (int n = 0; n < num_states; n++) {
      ref_stamps.push_back(fs_vehicle_traj[n].frenet_state.time_stamp);
      // 提取纵向s和横向l位置作为参考点
      ref_points.push_back(Vecf<2>(fs_vehicle_traj[n].frenet_state.vec_s[0],
                                   fs_vehicle_traj[n].frenet_state.vec_dt[0]));
      ref_states.push_back(fs_vehicle_traj[n].frenet_state);
    }

    /**
     * **Bézier样条生成与QP优化（核心函数）**
     * 
     * GetBezierSplineUsingCorridor() 实现的功能：
     * 
     * 1. **分段Bézier曲线生成（对应文档公式2）**：
     *    f_j^σ(t) = α_j · Σ p_i^j · b_m^i((t-t_{j-1})/α_j), t ∈ [t_{j-1}, t_j]
     *    - 每段Bézier曲线对应SSC走廊中的一个立方体
     *    - α_j 是第j段的时间缩放因子
     * 
     * 2. **代价函数优化（对应文档公式3）**：
     *    J_j = w_s ∫(d³f^s/dt³)²dt + w_l ∫(d³f^l/dt³)²dt
     *    - w_s, w_l：纵向和横向平滑度权重
     *    - 最小化加加速度（jerk）的平方积分，生成平滑轨迹
     *    - 可选：添加与参考状态的相似度代价项
     * 
     * 3. **约束施加**：
     *    - **边界约束（公式7）**：起始和终止状态约束（位置、速度、加速度）
     *    - **自由空间约束（公式6，k=0）**：β_{j,-}^σ ≤ f_j^σ(t) ≤ β_{j,+}^σ
     *      利用Bézier曲线的凸包特性，将控制点约束在立方体内即可保证轨迹无碰撞
     *    - **动力学约束（公式6，k=1,2）**：
     *      β_{j,-}^σ ≤ d^k f_j^σ(t)/dt^k ≤ β_{j,+}^σ（速度、加速度约束）
     *      利用Bézier曲线的速端曲线特性，约束导数曲线的控制点
     *    - **连续性约束（公式8）**：
     *      当前实现对相邻段施加位置、速度、加速度连续（k=0,1,2）
     *      即保证相邻段之间的 C² 连续性
     * 
     * **输入参数**：
     * - cube_list[i]：第i个行为的SSC走廊（时空语义立方体序列）
     * - start_constraints：起始边界约束（公式7）
     * - end_constraints：终止边界约束（公式7）
     * - ref_stamps, ref_points：参考状态的时间戳和位置（用于相似度代价）
     * - weight_proximity：相似度代价的权重（对应文档公式3中的w_f^σ）
     * 
     * **输出**：
     * - bezier_spline：优化后的分段Bézier样条
     */
    bool bezier_spline_gen_success = true;
    if (spline_generator.GetBezierSplineUsingCorridor(
            cube_list[i], start_constraints, end_constraints, ref_stamps,
            ref_points, cfg_.planner_cfg().weight_proximity(),
            &bezier_spline) != kSuccess) {
      if (is_lateral_independent_) {
        LOG(ERROR) << "[Ssc]fail: solver error for behavior "
                   << static_cast<int>(forward_behaviors_[i]);
        decimal_t t0 = cube_list[i].front().t_lb;
        for (auto& cube : cube_list[i]) {
          LOG(ERROR) << std::fixed << std::setprecision(3) << "[Ssc] t: ["
                     << cube.t_lb - t0 << ", " << cube.t_ub - t0 << "], x: ["
                     << cube.p_lb[0] << ", " << cube.p_ub[0] << "], y: ["
                     << cube.p_lb[1] << ", " << cube.p_ub[1] << "]";
        }
        LOG(ERROR) << "[Ssc]ref points: ";
        for (int k = 0; k < ref_stamps.size(); ++k) {
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << ref_stamps[k] << ", x: " << ref_points[k].x()
                     << ", y: " << ref_points[k].y();
        }
        LOG(ERROR) << "[Ssc]forward traj: ";
        for (int k = 0; k < forward_trajs_[i].size(); ++k) {
          auto v = forward_trajs_[i][k];
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << v.state().time_stamp
                     << ", x: " << v.state().vec_position.x()
                     << ", y: " << v.state().vec_position.y()
                     << ", v: " << v.state().velocity;
        }
        LOG(ERROR) << "[Ssc]ref lane range: [" << nav_lane_local_.begin()
                   << ", " << nav_lane_local_.end() << "]";

        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd velocity (" << start_constraints[1](0)
                   << ", " << start_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd acceleration (" << start_constraints[2](0)
                   << ", " << start_constraints[2](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd position (" << end_constraints[0](0) << ", "
                   << end_constraints[0](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd velocity (" << end_constraints[1](0) << ", "
                   << end_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End state stamp: "
                   << fs_vehicle_traj[num_states - 1].frenet_state.time_stamp;
      }
      bezier_spline_gen_success = false;
    }

    /**
     * **低速情况下的Primitive轨迹生成**
     * 
     * 当 is_lateral_independent_ = false 时（即低速情况），
     * 横向和纵向运动不能独立规划，需要使用Primitive轨迹连接方法。
     * 
     * **原因**：在低速情况下（如停车、起步），车辆运动学模型接近奇异点，
     * Frenet坐标系可能出现数值不稳定，此时使用简化的Primitive连接更可靠。
     * 
     * **物理意义**：在低速场景下，使用直线或简单曲线连接起始和目标状态，
     * 避免Bézier曲线优化在低速时的数值问题。
     */
    FrenetPrimitive primitive;
    if (!is_lateral_independent_) {
      // 使用Primitive连接起始和目标状态（低速场景的简化方法）
      primitive.Connect(initial_frenet_state_,
                        fs_vehicle_traj.back().frenet_state,
                        initial_frenet_state_.time_stamp,
                        fs_vehicle_traj.back().frenet_state.time_stamp -
                            initial_frenet_state_.time_stamp,
                        is_lateral_independent_);
    }

    // 在高速情况下，如果Bézier样条生成失败，跳过该行为
    if (is_lateral_independent_ && !bezier_spline_gen_success) continue;
    
    // 保存优化后的轨迹和相关信息
    qp_trajs_.push_back(bezier_spline);              // Bézier样条轨迹（公式2）
    primitive_trajs_.push_back(primitive);           // Primitive轨迹（低速备用）
    corridors_.push_back(cube_list[i]);              // 对应的SSC走廊
    ref_states_list_.push_back(ref_states);          // 参考状态列表
    valid_behaviors_.push_back(forward_behaviors_[i]); // 有效行为
  }
  return kSuccess;
}

/**
 * @brief 根据当前行为选择最终轨迹
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **轨迹选择策略**：
 * 1. **精确匹配**：优先选择与当前行为（ego_behavior_）完全匹配的轨迹
 *    - 如果行为规划器输出"左变道"，选择左变道对应的优化轨迹
 * 2. **备选策略**：如果精确匹配失败，选择车道保持（kLaneKeeping）轨迹
 *    - 车道保持是最安全的备选行为，通常总是可行
 * 
 * **输出轨迹**：
 * - trajectory_：主要轨迹，Frenet坐标系中的Bézier样条（文档公式2）
 *   - 通过FrenetBezierTrajectory将Frenet坐标转换为全局坐标
 *   - 高速情况下使用（is_lateral_independent_ = true）
 * 
 * - low_spd_alternative_traj_：低速备选轨迹，Primitive轨迹
 *   - 通过FrenetPrimitiveTrajectory将Frenet坐标转换为全局坐标
 *   - 低速情况下使用（is_lateral_independent_ = false）
 * 
 * - final_corridor_：最终选择的SSC走廊（文档第5节）
 * - final_ref_states_：最终选择的参考状态列表
 * 
 * **物理意义**：
 * 确保生成的轨迹与行为规划器的决策一致，实现行为规划与运动规划的紧密集成。
 */
ErrorType SscPlanner::UpdateTrajectoryWithCurrentBehavior() {
  int num_valid_behaviors = static_cast<int>(valid_behaviors_.size());
  if (num_valid_behaviors < 1) {
    return kWrongStatus;
  }
  
  // 策略1：查找与当前行为精确匹配的轨迹
  bool find_exact_match_behavior = false;
  int index = 0;
  for (int i = 0; i < num_valid_behaviors; i++) {
    if (valid_behaviors_[i] == ego_behavior_) {
      find_exact_match_behavior = true;
      index = i;
    }
  }
  
  // 策略2：如果精确匹配失败，查找车道保持轨迹作为备选
  bool find_candidate_behavior = false;
  LateralBehavior candidate_bahavior = common::LateralBehavior::kLaneKeeping;
  if (!find_exact_match_behavior) {
    for (int i = 0; i < num_valid_behaviors; i++) {
      if (valid_behaviors_[i] == candidate_bahavior) {
        find_candidate_behavior = true;
        index = i;
      }
    }
  }
  
  // 如果两种策略都失败，返回错误
  if (!find_exact_match_behavior && !find_candidate_behavior)
    return kWrongStatus;

  // 将Frenet坐标系中的Bézier样条转换为全局坐标轨迹（文档公式2）
  trajectory_ = FrenetBezierTrajectory(qp_trajs_[index], stf_);
  
  // 将Frenet坐标系中的Primitive轨迹转换为全局坐标轨迹（低速备选）
  low_spd_alternative_traj_ =
      FrenetPrimitiveTrajectory(primitive_trajs_[index], stf_);
  
  // 保存最终选择的走廊和参考状态
  final_corridor_ = corridors_[index];
  final_ref_states_ = ref_states_list_[index];
  return kSuccess;
}

/**
 * @brief 检查SSC走廊的可行性（时间连续性检查）
 * @param cubes SSC走廊中的立方体序列
 * @return 返回错误类型，kSuccess表示走廊可行
 * 
 * **对应文档公式8：连续性约束**
 * 
 * 公式(8)：\frac{d^k f_j^σ(t_j)}{dt^k} = \frac{d^k f_{j+1}^σ(t_j)}{dt^k}
 * 
 * 本函数检查相邻立方体之间的时间连续性，这是施加连续性约束的前提条件。
 * 
 * **检查内容**：
 * - 前一个立方体的时间上界 t_ub 必须等于下一个立方体的时间下界 t_lb
 * - cubes[i-1].t_ub == cubes[i].t_lb （对于所有相邻立方体对）
 * 
 * **物理意义**：
 * 1. **时间连续性**：确保相邻段之间时间无缝衔接，避免时间间隔或重叠
 * 2. **分段对应**：每个立方体对应一个Bézier曲线段，时间连续是段间连续的先决条件
 * 3. **约束前提**：只有在时间连续的前提下，才能施加位置、速度、加速度的连续性约束（公式8）
 * 
 * **与公式8的关系**：
 * - 论文中的连续性约束用于连接相邻段
 * - 当前优化器实现实际施加的是 k=0,1,2 的连续性约束
 * - 本函数确保 t_j 有明确定义（前一段的 t_ub = 后一段的 t_lb）
 * - 如果时间不连续，公式8的连续性约束无法正确施加
 * 
 * **失败原因**：
 * - SSC走廊生成算法可能产生时间不连续的立方体（算法bug）
 * - 前向仿真轨迹的时间戳可能不一致
 */
ErrorType SscPlanner::CorridorFeasibilityCheck(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes) {
  int num_cubes = static_cast<int>(cubes.size());
  if (num_cubes < 1) {
    LOG(ERROR) << "[Ssc]number of cubes not enough.";
    return kWrongStatus;
  }
  
  // 检查相邻立方体之间的时间连续性（公式8连续性约束的前提）
  // 要求：cubes[i-1].t_ub == cubes[i].t_lb（对所有相邻立方体对）
  for (int i = 1; i < num_cubes; i++) {
    if (cubes[i - 1].t_ub != cubes[i].t_lb) {
      LOG(ERROR) << "[Ssc]Err- Corridor not consist.";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i - 1].t_lb << ", "
                 << cubes[i - 1].t_ub << "], x: [" << cubes[i - 1].p_lb[0]
                 << ", " << cubes[i - 1].p_ub[0] << "], y: ["
                 << cubes[i - 1].p_lb[1] << ", " << cubes[i - 1].p_ub[1] << "]";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i].t_lb << ", " << cubes[i].t_ub
                 << "], x: [" << cubes[i].p_lb[0] << ", " << cubes[i].p_ub[0]
                 << "], y: [" << cubes[i].p_lb[1] << ", " << cubes[i].p_ub[1]
                 << "]";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/**
 * @brief 将输入数据从全局坐标系转换为Frenet坐标系
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **对应文档第5.1节：语义元素与Frenét坐标系表示**
 * 
 * 本函数实现三个阶段的数据转换流程：
 * 
 * **阶段I：数据打包**
 * 收集需要转换的所有全局坐标系数据：
 * 1. 自车状态和顶点：初始状态及其车辆轮廓顶点
 * 2. 自车前向仿真轨迹：每个行为的前向仿真轨迹状态和顶点
 * 3. 周围车辆轨迹：周围车辆的前向轨迹（用于SSC地图构建）
 * 4. 障碍物网格点：静态障碍物的占据网格点
 * 
 * **阶段II：坐标转换**
 * 将所有全局坐标数据批量转换为Frenet坐标系：
 * - 使用StateTransformer进行状态转换（位置、速度、加速度等）
 * - 支持OpenMP多线程并行转换（USE_OPENMP=1）或单线程转换（默认）
 * 
 * **阶段III：数据提取**
 * 将转换后的Frenet坐标系数据按原始结构重组：
 * - 提取自车的Frenet状态和顶点
 * - 提取每个行为的前向轨迹Frenet状态和顶点
 * - 提取周围车辆的Frenet轨迹
 * - 提取障碍物网格点的Frenet坐标
 * 
 * **Frenet坐标系优势**（文档第5.1节）：
 * 1. **语义元素关联**：限速、交通信号灯等与纵向位置 $s$ 直接相关
 * 2. **自然分解**：驾驶行为可分解为纵向 $s$ 和横向 $l$ 运动
 * 3. **自由空间建模**：在 slt 三维空间中建模更直观
 *    - 纵向 $s$：沿参考车道的弧长距离
 *    - 横向 $l$：垂直于参考车道方向的偏移
 *    - 时间 $t$：处理动态障碍物和时变约束
 * 
 * **转换后的数据用途**：
 * - forward_trajs_fs_：用于生成SSC走廊的种子（文档算法1）
 * - surround_forward_trajs_fs_：用于构建SSC地图中的动态障碍物（文档第5.1节）
 * - obstacle_grids_fs_：用于构建SSC地图中的静态障碍物（文档第5.1节）
 * - ego_frenet_state_：用于设置轨迹优化的起始边界约束（文档公式7）
 */
ErrorType SscPlanner::StateTransformForInputData() {
  vec_E<State> global_state_vec;
  vec_E<Vec2f> global_point_vec;
  int num_v;

  // ========== 阶段I：数据打包 ==========
  // 收集所有需要转换的全局坐标系数据
  
  /**
   * **自车状态和顶点打包**
   * 
   * 转换自车的初始状态及其车辆轮廓顶点（用于表示车辆占用空间）。
   * 车辆顶点用于碰撞检测和SSC地图构建。
   */
  // * Ego vehicle state and vertices
  {
    global_state_vec.push_back(initial_state_);
    vec_E<Vec2f> v_vec;
    // 获取车辆轮廓顶点（4个角点）
    common::SemanticsUtils::GetVehicleVertices(ego_vehicle_.param(),
                                               initial_state_, &v_vec);
    num_v = v_vec.size();  // 车辆顶点数量（通常为4）
    global_point_vec.insert(global_point_vec.end(), v_vec.begin(), v_vec.end());
  }

  /**
   * **自车前向仿真轨迹打包**
   * 
   * 转换每个行为对应的自车前向仿真轨迹（对应文档算法1的种子生成）。
   * 这些轨迹状态将作为SSC走廊生成的种子（文档算法1第3行）。
   */
  // * Ego forward simulation trajs states and vertices
  {
    common::VehicleParam ego_param = ego_vehicle_.param();
    for (int i = 0; i < (int)forward_trajs_.size(); ++i) {
      if (forward_trajs_[i].size() < 1) continue;
      for (int k = 0; k < (int)forward_trajs_[i].size(); ++k) {
        // 状态：包含位置、速度、加速度等信息
        State traj_state = forward_trajs_[i][k].state();
        global_state_vec.push_back(traj_state);
        // 顶点：车辆在该状态下的轮廓顶点
        vec_E<Vec2f> v_vec;
        common::SemanticsUtils::GetVehicleVertices(ego_param, traj_state,
                                                   &v_vec);
        global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                v_vec.end());
      }
    }
  }

  /**
   * **周围车辆轨迹打包（对应文档第5.1节：类障碍物语义元素）**
   * 
   * 转换周围车辆的前向轨迹，用于构建SSC地图中的动态障碍物。
   * 根据文档第5.1节，动态障碍物可根据预测轨迹视为时间域中的一系列静态障碍物。
   * 
   * 注意：这些轨迹来自行为规划器（如MPDM）的前向仿真，已考虑了交互。
   */
  // * Surrounding vehicle trajs from MPDM
  {
    for (int i = 0; i < surround_forward_trajs_.size(); ++i) {
      for (auto it = surround_forward_trajs_[i].begin();
           it != surround_forward_trajs_[i].end(); ++it) {
        for (int k = 0; k < it->second.size(); ++k) {
          // 周围车辆的状态
          State traj_state = it->second[k].state();
          global_state_vec.push_back(traj_state);
          // 周围车辆的轮廓顶点
          vec_E<Vec2f> v_vec;
          common::SemanticsUtils::GetVehicleVertices(it->second[k].param(),
                                                     traj_state, &v_vec);
          global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                  v_vec.end());
        }
      }
    }
  }

  /**
   * **障碍物网格点打包（对应文档第5.1节：类障碍物语义元素）**
   * 
   * 转换静态障碍物的占据网格点。
   * 根据文档第5.1节，静态障碍物可视为跨越整个时间轴的障碍物。
   */
  // * Obstacle grids
  {
    for (auto it = obstacle_grids_.begin(); it != obstacle_grids_.end(); ++it) {
      Vec2f pt((*it)[0], (*it)[1]);
      global_point_vec.push_back(pt);
    }
  }

  // ========== 阶段II：坐标转换 ==========
  // 预分配Frenet坐标系输出向量
  vec_E<FrenetState> frenet_state_vec(global_state_vec.size());
  vec_E<Vec2f> fs_point_vec(global_point_vec.size());

  /**
   * **批量坐标转换**
   * 
   * 使用StateTransformer将所有全局坐标数据转换为Frenet坐标系。
   * - 状态转换：State → FrenetState（包含纵向s、横向l、速度、加速度等）
   * - 点转换：Vec2f(x, y) → Vec2f(s, l)
   * 
   * 支持OpenMP多线程并行转换以提升性能（当USE_OPENMP=1时）。
   */
#if USE_OPENMP
  // OpenMP多线程并行转换（适用于大量数据）
  TicToc timer_stf;
  StateTransformUsingOpenMp(global_state_vec, global_point_vec,
                            &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]OpenMp transform time cost: " << timer_stf.toc()
               << " ms.";
#else
  // 单线程转换（默认方式）
  TicToc timer_stf;
  StateTransformSingleThread(global_state_vec, global_point_vec,
                             &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]Single thread transform time cost: " << timer_stf.toc()
               << " ms.";
#endif

  // ========== 阶段III：数据提取 ==========
  // 按原始结构重组转换后的Frenet坐标系数据
  
  int offset = 0;
  
  /**
   * **提取自车的Frenet状态和顶点**
   * 
   * 自车的Frenet状态将用于：
   * 1. 设置轨迹优化的起始边界约束（文档公式7）
   * 2. 初始化SSC地图（ResetSscMap）
   */
  // * Ego vehicle state and vertices
  {
    fs_ego_vehicle_.frenet_state = frenet_state_vec[offset];
    fs_ego_vehicle_.vertices.clear();
    for (int i = 0; i < num_v; ++i) {
      fs_ego_vehicle_.vertices.push_back(fs_point_vec[offset * num_v + i]);
    }
    offset++;
  }

  /**
   * **提取自车前向仿真轨迹的Frenet状态和顶点**
   * 
   * 这些轨迹将作为SSC走廊生成的种子（文档算法1第3行），
   * 并用于提取参考状态（RunQpOptimization中的ref_states）。
   */
  // * Ego forward simulation trajs states and vertices
  {
    forward_trajs_fs_.clear();
    if (forward_trajs_.size() < 1) return kWrongStatus;
    for (int j = 0; j < (int)forward_trajs_.size(); ++j) {
      if (forward_trajs_[j].size() < 1) assert(false);
      vec_E<common::FsVehicle> traj_fs;
      for (int k = 0; k < (int)forward_trajs_[j].size(); ++k) {
        common::FsVehicle fs_v;
        fs_v.frenet_state = frenet_state_vec[offset];
        for (int i = 0; i < num_v; ++i) {
          fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
        }
        traj_fs.emplace_back(fs_v);
        offset++;
      }
      forward_trajs_fs_.emplace_back(traj_fs);
    }
  }

  /**
   * **提取周围车辆轨迹的Frenet状态和顶点**
   * 
   * 这些轨迹将用于构建SSC地图中的动态障碍物（文档第5.1节）。
   * 每个行为对应一组周围车辆的轨迹（因为不同行为下其他车辆的响应不同）。
   */
  // * Surrounding vehicle trajs from MPDM
  {
    surround_forward_trajs_fs_.clear();
    for (int j = 0; j < surround_forward_trajs_.size(); ++j) {
      std::unordered_map<int, vec_E<common::FsVehicle>> sur_trajs;
      for (auto it = surround_forward_trajs_[j].begin();
           it != surround_forward_trajs_[j].end(); ++it) {
        int v_id = it->first;
        vec_E<common::FsVehicle> traj_fs;
        for (int k = 0; k < it->second.size(); ++k) {
          common::FsVehicle fs_v;
          fs_v.frenet_state = frenet_state_vec[offset];
          for (int i = 0; i < num_v; ++i) {
            fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
          }
          traj_fs.emplace_back(fs_v);
          offset++;
        }
        sur_trajs.insert(
            std::pair<int, vec_E<common::FsVehicle>>(v_id, traj_fs));
      }
      surround_forward_trajs_fs_.emplace_back(sur_trajs);
    }
  }

  /**
   * **提取障碍物网格点的Frenet坐标**
   * 
   * 这些网格点将用于构建SSC地图中的静态障碍物（文档第5.1节）。
   * 静态障碍物可视为跨越整个时间轴的障碍物。
   */
  // * Obstacle grids
  {
    obstacle_grids_fs_.clear();
    for (int i = 0; i < static_cast<int>(obstacle_grids_.size()); ++i) {
      obstacle_grids_fs_.push_back(fs_point_vec[offset * num_v + i]);
    }
  }

  // 保存自车的Frenet状态（用于后续优化）
  ego_frenet_state_ = fs_ego_vehicle_.frenet_state;
  return kSuccess;
}

/**
 * @brief 使用OpenMP多线程并行进行坐标转换
 * @param global_state_vec 全局坐标系状态向量
 * @param global_point_vec 全局坐标系点向量
 * @param frenet_state_vec 输出：Frenét坐标系状态向量
 * @param fs_point_vec 输出：Frenét坐标系点向量
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 使用OpenMP多线程并行将全局坐标系数据转换为Frenét坐标系。
 * 适用于大量数据转换的场景，可显著提升性能。
 * 
 * **转换内容**：
 * - 状态转换：State → FrenetState（包含纵向s、横向l、速度、加速度等）
 * - 点转换：Vec2f(x, y) → Vec2f(s, l)
 * 
 * **并行策略**：
 * - 使用4个线程并行处理状态和点的转换
 * - 每个线程独立调用StateTransformer进行转换
 * 
 * **注意**：当前代码中USE_OPENMP=0，默认使用单线程转换。
 * 如需启用多线程，需设置USE_OPENMP=1。
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
ErrorType SscPlanner::StateTransformUsingOpenMp(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  // 获取状态和点的数量，用于后续的并行循环
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();

  // 获取输出向量的原始指针，用于高效的并行写入
  // 使用指针直接写入可以避免OpenMP并行时的竞争条件
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();

  // 记录需要转换的总查询数量（用于性能监控）
  LOG(WARNING) << "[Ssc]OpenMp - Total number of queries: "
               << state_num + point_num;
  
  // 设置OpenMP线程数为4（可根据CPU核心数调整）
  omp_set_num_threads(4);
  
  /**
   * **阶段1：并行转换状态向量**
   * 
   * 将全局坐标系的状态（State）转换为Frenét坐标系的状态（FrenetState）。
   * 
   * **转换内容**（对应文档第5.1节：Frenét坐标系定义）：
   * - 位置：(x, y) → (s, l)
   *   - s：沿参考车道的弧长距离（纵向位置）
   *   - l：垂直于参考车道方向的偏移距离（横向位置）
   * - 速度：全局速度 → Frenét速度（纵向速度 s_dot、横向速度 l_dot）
   * - 加速度：全局加速度 → Frenét加速度
   * - 时间戳：保持不变
   * 
   * **并行策略**：
   * - #pragma omp parallel for：OpenMP自动将循环分配给多个线程
   * - 每个线程独立处理不同的状态转换，无数据竞争
   * - 使用原始指针写入，避免向量操作的线程安全问题
   */
  {
#pragma omp parallel for
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      // 调用StateTransformer进行坐标转换
      // 如果转换失败，至少保留时间戳信息
      if (kSuccess != stf_.GetFrenetStateFromState(global_state_vec[i], &fs)) {
        // 转换失败时，保留原始时间戳以确保时间一致性
        fs.time_stamp = global_state_vec[i].time_stamp;
      }
      // 将转换后的Frenét状态写入输出向量（使用指针避免竞争）
      *(ptr_state_vec + i) = fs;
    }
  }
  
  /**
   * **阶段2：并行转换点向量**
   * 
   * 将全局坐标系的点（Vec2f(x, y)）转换为Frenét坐标系的点（Vec2f(s, l)）。
   * 
   * **转换内容**：
   * - 二维点：(x, y) → (s, l)
   *   - 用于转换车辆轮廓顶点、障碍物网格点等几何信息
   * 
   * **用途**（对应文档第5.1节）：
   * - 车辆轮廓顶点：用于SSC地图构建中的碰撞检测
   * - 障碍物网格点：用于构建静态障碍物占据网格
   * 
   * **并行策略**：
   * - 与状态转换类似，使用OpenMP并行处理
   * - 每个点的转换相互独立，适合并行化
   */
  {
#pragma omp parallel for
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      // 调用StateTransformer进行点坐标转换
      // 将全局坐标点(x, y)转换为Frenét坐标点(s, l)
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      // 将转换后的Frenét点写入输出向量
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

/**
 * @brief 使用单线程进行坐标转换（默认方式）
 * @param global_state_vec 全局坐标系状态向量
 * @param global_point_vec 全局坐标系点向量
 * @param frenet_state_vec 输出：Frenét坐标系状态向量
 * @param fs_point_vec 输出：Frenét坐标系点向量
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 使用单线程将全局坐标系数据转换为Frenét坐标系。
 * 这是默认的转换方式，适用于大多数场景。
 * 
 * **转换内容**：
 * - 状态转换：State → FrenetState（包含纵向s、横向l、速度、加速度等）
 * - 点转换：Vec2f(x, y) → Vec2f(s, l)
 * 
 * **性能**：
 * - 对于中等规模的数据，单线程性能已足够
 * - 避免了多线程的开销和同步问题
 * 
 * **对应文档**：第5.1节（语义元素与Frenét坐标系表示）
 */
ErrorType SscPlanner::StateTransformSingleThread(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  // 获取状态和点的数量
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();
  
  // 获取输出向量的原始指针（用于高效写入）
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();
  
  /**
   * **阶段1：单线程转换状态向量**
   * 
   * 将全局坐标系的状态（State）转换为Frenét坐标系的状态（FrenetState）。
   * 
   * **转换内容**（对应文档第5.1节：Frenét坐标系定义）：
   * - 位置：(x, y) → (s, l)
   *   - s：沿参考车道的弧长距离（纵向位置）
   *   - l：垂直于参考车道方向的偏移距离（横向位置）
   * - 速度：全局速度 → Frenét速度（纵向速度 s_dot、横向速度 l_dot）
   * - 加速度：全局加速度 → Frenét加速度
   * 
   * **单线程优势**：
   * - 简单直接，无线程同步开销
   * - 适合中等规模数据转换
   * - 避免多线程的调度开销
   */
  {
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      // 调用StateTransformer进行坐标转换
      // 将全局坐标系状态转换为Frenét坐标系状态
      stf_.GetFrenetStateFromState(global_state_vec[i], &fs);
      // 将转换后的Frenét状态写入输出向量
      *(ptr_state_vec + i) = fs;
    }
  }
  
  /**
   * **阶段2：单线程转换点向量**
   * 
   * 将全局坐标系的点（Vec2f(x, y)）转换为Frenét坐标系的点（Vec2f(s, l)）。
   * 
   * **转换内容**：
   * - 二维点：(x, y) → (s, l)
   *   - 用于转换车辆轮廓顶点、障碍物网格点等几何信息
   * 
   * **用途**（对应文档第5.1节：类障碍物语义元素）：
   * - 车辆轮廓顶点：用于SSC地图构建中的碰撞检测
   * - 障碍物网格点：用于构建静态障碍物占据网格
   *    - 静态障碍物可视为跨越整个时间轴的障碍物（文档第5.1节）
   * 
   * **性能考虑**：
   * - 对于中等规模数据（如几百个点），单线程性能足够
   * - 如果数据量很大（数千个点），可考虑使用OpenMP多线程版本
   */
  {
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      // 调用StateTransformer进行点坐标转换
      // 将全局坐标点(x, y)转换为Frenét坐标点(s, l)
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      // 将转换后的Frenét点写入输出向量
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

/**
 * @brief 设置地图接口
 * @param map_itf 地图接口指针，提供环境信息（障碍物、车道、前向轨迹等）
 * @return 返回错误类型，kSuccess表示成功
 * 
 * **功能**：
 * 设置规划器与上层系统（如行为规划层、感知层）的接口。
 * 地图接口提供规划所需的所有环境信息。
 * 
 * **地图接口提供的信息**（对应文档第3节：系统概述）：
 * 1. **自车信息**：当前自车状态、车辆参数
 * 2. **参考车道**：局部参考车道（用于构建Frenét坐标系）
 * 3. **障碍物信息**：静态障碍物地图和网格点
 * 4. **前向轨迹**：自车和周围车辆的前向仿真轨迹（来自行为规划器）
 * 5. **行为信息**：当前自车的离散行为（车道保持、左变道、右变道等）
 * 
 * **接口设计**：
 * - 采用接口模式，解耦规划器与具体的地图实现
 * - 便于在不同系统间复用规划器代码
 * 
 * **对应文档**：第3节（系统概述）
 */
ErrorType SscPlanner::set_map_interface(SscPlannerMapItf* map_itf) {
  if (map_itf == nullptr) return kIllegalInput;
  map_itf_ = map_itf;
  map_valid_ = true;
  return kSuccess;
}

/**
 * @brief 验证生成的轨迹的有效性
 * @param traj 待验证的轨迹
 * @return 返回错误类型，kSuccess表示轨迹有效
 * 
 * **验证内容**（确保轨迹满足基本要求）：
 * 
 * 1. **起始状态验证（对应文档公式7：期望状态约束）**：
 *    - **位置匹配**：轨迹起始位置应与初始状态匹配（容差0.1m）
 *      - 验证：||position(t_0) - initial_position|| ≤ 0.1
 *      - 对应公式7的k=0约束：f^σ(t_0) = σ_{t_0}^{(0)}
 * 
 *    - **速度匹配**：轨迹起始速度应与初始状态匹配（容差0.1m/s）
 *      - 验证：|velocity(t_0) - initial_velocity| ≤ 0.1
 *      - 对应公式7的k=1约束：df^σ(t_0)/dt = σ_{t_0}^{(1)}
 * 
 * **物理意义**：确保生成的Bézier轨迹正确满足起始边界约束（公式7），
 * 实现平滑的轨迹起始。
 * 
 * 2. **终止状态验证**：
 *    - 检查轨迹终止状态是否可以正确评估
 *    - 确保轨迹在终止时间点有效
 * 
 * 3. **动力学可行性验证**：
 *    - **曲率约束**：检查轨迹的曲率是否在合理范围内
 *      - 约束：|curvature(t)| ≤ 0.33 rad/m（约19度/米）
 *      - **物理意义**：过大的曲率会导致车辆无法跟踪，或需要过大的转向角
 *      - **对应文档**：虽然文档公式6约束了速度和加速度，但曲率是轨迹的
 *        固有几何特性，需要额外验证
 * 
 * **验证方法**：
 * - 在轨迹时间范围内以0.1秒间隔采样
 * - 对每个采样点评估轨迹状态
 * - 检查状态评估是否成功，以及曲率是否在允许范围内
 * 
 * **失败原因**：
 * - 边界约束不匹配（公式7未正确施加）
 * - 轨迹优化失败（QP求解器未找到可行解）
 * - 动力学约束过于宽松（导致不可跟踪的轨迹）
 * 
 * **注意**：当前代码中此函数被禁用（#if 0），
 * 但保留用于调试和验证轨迹生成的正确性。
 */
ErrorType SscPlanner::ValidateTrajectory(const FrenetTrajectory& traj) {
  // 在轨迹时间范围内以0.1秒间隔采样（用于验证）
  std::vector<decimal_t> t_vec_xy;
  common::GetRangeVector<decimal_t>(traj.begin(), traj.end(), 0.1, true,
                                    &t_vec_xy);
  common::State state;
  
  /**
   * **起始状态验证（对应文档公式7：期望状态约束）**
   * 
   * 验证轨迹是否满足起始边界约束：
   * - 位置约束：f^σ(t_0) = σ_{t_0}^{(0)}（公式7，k=0）
   * - 速度约束：df^σ(t_0)/dt = σ_{t_0}^{(1)}（公式7，k=1）
   */
  // * check init state
  if (traj.GetState(traj.begin(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]State evaluation error";
    return kWrongStatus;
  }

  // 验证起始位置匹配（容差0.1m）
  // 对应公式7：f^σ(t_0) = σ_{t_0}^{(0)}（位置约束）
  if ((state.vec_position - initial_state_.vec_position).norm() > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init position miss match";
    return kWrongStatus;
  }

  // 验证起始速度匹配（容差0.1m/s）
  // 对应公式7：df^σ(t_0)/dt = σ_{t_0}^{(1)}（速度约束）
  if (fabs(state.velocity - initial_state_.velocity) > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init vel miss match";
    return kWrongStatus;
  }
  
  // * check end state
  // 验证终止状态可以正确评估
  if (traj.GetState(traj.end(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]End state eval error";
    return kWrongStatus;
  }

  /**
   * **动力学可行性验证**
   * 
   * 在轨迹上采样点验证曲率约束，确保轨迹可被车辆跟踪。
   * 
   * 曲率约束：|curvature| ≤ 0.33 rad/m（约19度/米）
   * - 过大的曲率会导致车辆无法跟踪
   * - 或需要过大的转向角，超出车辆物理限制
   */
  for (const auto t : t_vec_xy) {
    // 评估轨迹在采样点的状态
    if (traj.GetState(t, &state) != kSuccess) {
      LOG(ERROR) << "[Ssc][Validate]State eval error";
      return kWrongStatus;
    }
    
    // 检查曲率是否在合理范围内（≤ 0.33 rad/m）
    if (fabs(state.curvature) > 0.33) {
      LOG(ERROR) << "[Ssc][Validate]initial_state velocity "
                 << initial_state_.velocity << " Curvature " << state.curvature
                 << " invalid.";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

}  // namespace planning
