# slam_toolbox serialized maps

Localization mode requires a serialized pose graph, not only an occupancy-grid
`.yaml` and `.pgm`.

After completing a slam_toolbox mapping run, save the graph with:

```bash
mkdir -p /home/hcmute/agv_mpc/maps/slam_toolbox
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: {data: '/home/hcmute/agv_mpc/maps/slam_toolbox/maze_31'}}"
```

The command must create both:

```text
maze_31.posegraph
maze_31.data
```

Use the basename when launching localization:

```bash
map_file_name:=/home/hcmute/agv_mpc/maps/slam_toolbox/maze_31
```
