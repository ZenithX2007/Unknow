/**
 * @file marcos.h
 * @author GW
 * @brief 公共宏定义头文件
 * @version 0.1
 * @date 2019-03-17
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件集中放置项目范围内复用的基础宏定义。
 */
#ifndef _COMMON_INC_COMMON_MACROS_H__
#define _COMMON_INC_COMMON_MACROS_H__
#define DECLARE_BACKWARD       \
  namespace backward {         \
  backward::SignalHandling sh; \
  }
#endif
