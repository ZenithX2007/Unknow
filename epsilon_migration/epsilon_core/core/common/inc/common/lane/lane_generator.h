#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_GENERATOR_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_GENERATOR_H__

#include "common/lane/lane.h"

#include "common/basics/basics.h"
#include "common/basics/config.h"

namespace common {
/**
 * @brief 从离散中心线点生成可查询的 Lane 样条
 *
 * 输入样本通常来自地图车道段、导航路径拼接或局部车道检测；
 * 输出 Lane 被语义地图、前向仿真和 SSC 共同作为 Frenet 参考线。
 */
class LaneGenerator {
 public:
  // 按调用方给定的参数序列插值，适合已知弧长或归一化参数的样本。
  static ErrorType GetLaneBySampleInterpolation(
      const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
      Lane* lane);

  // 自动依据采样点生成参数并插值，是最常用的车道构建入口。
  static ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                         Lane* lane);

  /**
   * @brief 以带连续性约束的最小二乘方式拟合车道
   * @note regulator 越大越强调相邻样条段的平滑连续，推荐范围 [1e6, 1e8)
   */

  static ErrorType GetLaneBySampleFitting(const vec_Vecf<LaneDim>& samples,
                                          const std::vector<decimal_t>& para,
                                          const Eigen::ArrayXf& breaks,
                                          const decimal_t regulator,
                                          Lane* lane);
};

}  // namespace common

#endif
