# Third-Party Code

## m-explore ROS 2

Local path:

```text
gen0_gz_sim_ros2/m-explore-ros2/
```

Purpose:

```text
explore_lite frontier exploration
explore_lite_msgs status messages
```

License:

```text
BSD license, see gen0_gz_sim_ros2/m-explore-ros2/LICENSE
```

Local cleanup:

```text
Removed unused map_merge package.
Removed TurtleBot demo files that came with map_merge.
Removed nested .git, .github, and .devcontainer directories.
Kept explore, explore_lite_msgs, README.md, and LICENSE.
```

Current demo dependency:

```text
sweeper_integration/launch/explore_gen0.launch.py
-> explore_lite executable
-> explore_lite_msgs/msg/ExploreStatus
```
