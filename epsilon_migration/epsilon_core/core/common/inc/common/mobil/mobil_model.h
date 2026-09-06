#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/rss/rss_checker.h"

namespace common {

/**
 * @brief MOBIL 换道评估的加速度变化计算
 *
 * MOBIL 不直接生成轨迹，而是比较换道前后 ego、原车道后车和
 * 目标车道后车的 IDM 加速度变化。上层预测器据此形成横向行为概率，
 * EUDM/MPDM 再将该语义信息用于场景推演。
 */
class MobilLaneChangingModel {
 public:
  // 当前车道：计算 ego 换出后，原后车及 ego 自身的加速度变化。
  static ErrorType GetMobilAccChangesOnCurrentLane(
      const FrenetState &cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
      decimal_t *acc_c);

  // 目标车道：先用 RSS 判定目标间隙，再计算 ego 插入后目标后车的影响。
  static ErrorType GetMobilAccChangesOnTargetLane(
      const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
      decimal_t *acc_n_tilda, decimal_t *acc_c_tilda);

 private:
  // 用真实或虚拟前车调用 IDM，统一处理“无前车”的自由流情况。
  static ErrorType GetDesiredAccelerationUsingIdm(
      const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
      const FrenetState &front_fs, const bool &use_virtual_front,
      decimal_t *acc);
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
