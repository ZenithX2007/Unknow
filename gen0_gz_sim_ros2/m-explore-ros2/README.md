# Vendored m-explore ROS 2

This directory keeps only the third-party packages used by the current Gen0
autonomous mapping demo:

```text
explore/
explore_lite_msgs/
LICENSE
```

`explore_lite` provides frontier exploration goals for Nav2. The unused
`map_merge`, TurtleBot simulation examples, nested `.git`, CI, and devcontainer
files were removed from this local vendor copy to keep the workspace focused.

The upstream code is BSD licensed. Keep `LICENSE` with this directory when
redistributing or submitting the project.
