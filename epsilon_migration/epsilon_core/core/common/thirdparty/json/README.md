# JSON 第三方库说明

## 1. 目录作用

本目录保存的是工程内嵌的 JSON 头文件库，用于在不额外安装系统包的前提下，直接在 C++ 代码中解析和生成 JSON 数据。

当前目录结构非常简单：

```text
core/common/thirdparty/json/
├── json.hpp
└── README.md
```

其中：

- [`json.hpp`](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/common/thirdparty/json/json.hpp) 是实际使用的单头文件库
- [`README.md`](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/common/thirdparty/json/README.md) 是本说明文档

## 2. 库来源

该文件是 `nlohmann/json` 的单头文件版本，也就是常见的 “JSON for Modern C++”。

从头文件顶部注释可以确认当前版本为：

```text
version 3.7.3
```

许可证为 MIT License。

## 3. 本工程为什么要内嵌它

在这个仓库里，很多配置文件和场景文件都以 JSON 形式保存，例如：

- playground 地图障碍物
- 车辆初始状态
- 语义地图管理器配置

如果每台机器都依赖外部包管理器安装 JSON 库，会增加环境差异和编译复杂度。把头文件直接放进仓库，有几个明显好处：

- 零运行时依赖
- 零链接依赖
- 只要包含头文件即可使用
- 更适合 catkin / ROS1 这种偏源码集成的项目

## 4. 本工程中的实际使用位置

目前仓库里我确认到的直接使用点主要有两个：

[arena_loader.h](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/phy_simulator/inc/phy_simulator/arena_loader.h)

- 用于读取仿真场景中的车辆、障碍物、车道网络 JSON 文件

[config_loader.h](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/semantic_map_manager/inc/semantic_map_manager/config_loader.h)

- 用于读取 `semantic_map_manager` 的智能体配置文件

这两个文件都采用同一种包含方式：

```cpp
#include <json/json.hpp>
```

## 5. 为什么 include 写成 `<json/json.hpp>`

这不是系统自带路径，而是工程自己的 include 目录配置决定的。

在 [core/common/CMakeLists.txt](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/common/CMakeLists.txt) 中：

- `thirdparty` 目录被加入了 `INCLUDE_DIRS`
- 因此编译器可以从 `core/common/thirdparty/` 下面找到 `json/json.hpp`

所以源码里可以直接写：

```cpp
#include <json/json.hpp>
```

而不需要写相对路径：

```cpp
#include "core/common/thirdparty/json/json.hpp"
```

这种写法的好处是：

- 代码更整洁
- 调整工程目录时影响更小
- 更接近“库”的使用方式

## 6. 如何在本工程中新代码里使用

如果你在本仓库的新模块里需要读写 JSON，推荐直接复用这份头文件库。

### 6.1 头文件包含

```cpp
#include <json/json.hpp>
```

### 6.2 常用别名

为了减少代码长度，通常建议定义一个别名：

```cpp
using json = nlohmann::json;
```

### 6.3 读取 JSON 文件示例

```cpp
#include <fstream>
#include <iostream>
#include <json/json.hpp>

using json = nlohmann::json;

bool LoadConfig(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    std::cerr << "failed to open: " << path << std::endl;
    return false;
  }

  json root;
  ifs >> root;

  double desired_vel = root["desired_vel"].get<double>();
  int ego_id = root["ego_id"].get<int>();

  std::cout << "desired_vel = " << desired_vel << std::endl;
  std::cout << "ego_id = " << ego_id << std::endl;
  return true;
}
```

### 6.4 生成 JSON 示例

```cpp
#include <json/json.hpp>

using json = nlohmann::json;

json BuildVehicleInfo() {
  json j;
  j["id"] = 0;
  j["type"] = "ego";
  j["state"] = {
      {"x", 10.0},
      {"y", 2.0},
      {"yaw", 0.1},
      {"vel", 8.5}
  };
  return j;
}
```

### 6.5 写回文件示例

```cpp
#include <fstream>
#include <json/json.hpp>

using json = nlohmann::json;

void SaveJson(const std::string& path, const json& j) {
  std::ofstream ofs(path);
  ofs << j.dump(2) << std::endl;  // 2 表示缩进空格数
}
```

## 7. 使用时的注意事项

### 7.1 这是头文件库，不需要链接

这份库是单头文件形式，因此：

- 不需要单独编译
- 不需要 `target_link_libraries(... json ...)`
- 只要 include 路径正确即可

### 7.2 读取字段时要注意异常

像下面这种写法：

```cpp
double v = root["desired_vel"].get<double>();
```

如果：

- 字段不存在
- 字段类型不匹配

就可能抛出异常。

因此在工程代码里，更稳妥的写法通常是先判断：

```cpp
if (root.contains("desired_vel") && root["desired_vel"].is_number()) {
  double v = root["desired_vel"].get<double>();
}
```

如果当前仓库某些旧代码没有这样做，后续扩展 JSON 配置时要格外小心。

### 7.3 不要和其他 JSON 库混用风格

本工程这里使用的是 `nlohmann::json`，它和旧式 `jsoncpp` 的接口风格完全不同。

例如：

`nlohmann::json` 风格：

```cpp
json root;
root["name"] = "ego";
std::string name = root["name"].get<std::string>();
```

`jsoncpp` 风格通常会写成：

```cpp
Json::Value root;
root["name"] = "ego";
std::string name = root["name"].asString();
```

不要把这两种接口混写。

## 8. 本工程中它适合用来做什么

结合当前仓库结构，这个库最适合处理以下类型的数据：

- 场景描述文件
- 配置文件
- 地图元素描述
- 调试导出数据
- 简单日志快照

不太建议把它直接用在以下高频实时路径上：

- 超高频控制回路里的频繁序列化
- 大规模二进制数据传输
- 对性能极端敏感的热点循环

原因不是它不能用，而是 JSON 本身和动态对象模型更适合“配置/交换/调试”，不适合做极限性能的数据通道。

## 9. 与本工程 CMake 的关系

这份库能被其他模块使用，根本原因是 `common` 包导出了 thirdparty include 路径。

关键位置见：

[core/common/CMakeLists.txt](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/common/CMakeLists.txt)

其中：

- `THIRD_PARTY_INCLUDE_DIRS` 包含 `core/common/thirdparty`
- `catkin_package(INCLUDE_DIRS ...)` 导出了这个路径

因此像下面这些模块都能间接使用它：

- [core/phy_simulator/CMakeLists.txt](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/phy_simulator/CMakeLists.txt)
- [core/semantic_map_manager/CMakeLists.txt](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/semantic_map_manager/CMakeLists.txt)

如果以后新增模块也想用 `<json/json.hpp>`，要确保该模块的 include 路径最终能拿到 `common` 的导出目录，或者显式加入 `core/common/thirdparty`。

## 10. 升级这个库时要注意什么

如果后续想升级 `json.hpp`，建议不要只替换头文件就结束，至少要检查下面几件事：

1. 当前源码中的包含路径是否仍兼容  
   也就是 `<json/json.hpp>` 这层目录结构要保持不变。

2. C++ 标准要求是否变化  
   当前工程整体是 `C++11`，升级后必须确认新版本仍支持当前编译选项。

3. 异常行为是否变化  
   某些版本升级后，`contains()`、`at()`、隐式转换、数字解析细节可能不同。

4. 编译时间是否明显变长  
   单头文件库升级后有时会增加模板展开和编译时间。

5. 现有 JSON 文件是否仍能被正确解析  
   至少应回归验证 playground、地图和 agent 配置文件读取流程。

## 11. 维护建议

如果你只是要在本工程里继续使用 JSON，建议遵循下面几条：

- 优先复用这份现有头文件，不要再引入第二套 JSON 库
- 新代码统一使用 `using json = nlohmann::json;`
- 配置解析前先检查字段存在性和类型
- 复杂 JSON 格式尽量在 README 或注释里写清字段含义
- 如果修改 `json.hpp` 版本，务必重新全量编译并回归读取场景文件

## 12. 许可证

当前目录中的 [`json.hpp`](/media/kinowu/B4CE107FCE103BD4/Plan_Control_Ros1_Sim/EPSILON-master/core/common/thirdparty/json/json.hpp) 来自 `nlohmann/json` 项目，采用 MIT License。

如需进一步查看完整许可证与项目来源，可直接阅读头文件顶部注释。
