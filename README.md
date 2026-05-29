# ROS 2 LiDAR Timestamp Bridge

Tento workspace obsahuje malý ROS 2 balík, ktorý posiela časové značky LiDAR správ do zdieľanej pamäte pre workspace synchronizácie kamery.

`lidar_stamp_bridge_node` sa prihlási na odber `/lidar_points`, z každej správy `sensor_msgs/msg/PointCloud2` číta `msg->header.stamp` a zapisuje časovú značku spolu s metadátami prijatia do `/dev/shm/lidar_stamp.bin`. Kamerový node v `ROS2-Camera-Sync-dominik` číta tento mmap súbor a vykonáva samotné párovanie časových značiek medzi kamerou a LiDARom.

Tento bridge nepáruje snímky z kamery so snímkami z LiDARu. Iba pripravuje posledné LiDAR časové značky pre kamerový workspace.

## Balík

```text
src/lidar_stamp_bridge
```

Nainštalované súbory:

- spustiteľný súbor: `lidar_stamp_bridge_node`
- launch súbor: `lidar_stamp_bridge.launch.py`
- konfiguračné súbory: `lidar_stamp_bridge.yaml`, `cyclonedds.xml`

## Požiadavky

- ROS 2 s `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- Cyclone DDS runtime pri použití dodaného launch súboru:

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

## Kompilácia

```bash
cd ROS2-LIDAR-CAMERA-BRIDGE
colcon build
source install/setup.bash
```

Ak sa veľké LiDAR point cloudy zahadzujú alebo Cyclone DDS hlási malý prijímací buffer, pred spustením zväčši kernel receive buffer:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
```

## Priame spustenie

Priame `ros2 run` nenačíta DDS prostredie z launch súboru. Ak LiDAR publisher tiež používa Cyclone DDS, najprv sourcni pomocný skript:

```bash
source src/lidar_stamp_bridge/scripts/source_cyclone_env.sh
```

Potom spusti node:

```bash
ros2 run lidar_stamp_bridge lidar_stamp_bridge_node --ros-args \
  -p topic:=/lidar_points \
  -p mmap_path:=/dev/shm/lidar_stamp.bin
```

## Spustenie cez launch

```bash
ros2 launch lidar_stamp_bridge lidar_stamp_bridge.launch.py
```

Launch súbor načíta parametre z:

```text
src/lidar_stamp_bridge/config/lidar_stamp_bridge.yaml
```

Zároveň nastaví:

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://<installed-package-share>/config/cyclonedds.xml
```

## Parametre

```yaml
lidar_stamp_bridge_node:
  ros__parameters:
    topic: "/lidar_points"
    mmap_path: "/dev/shm/lidar_stamp.bin"
    mmap_slots: 512
```

- `topic`: PointCloud2 topic, na ktorý sa node prihlási na odber.
- `mmap_path`: súbor v zdieľanej pamäti, do ktorého tento bridge zapisuje a z ktorého kamerový node číta.
- `mmap_slots`: počet záznamov časových značiek uložených v ring bufferi zdieľanej pamäte.

Subscription používa reliable QoS s hĺbkou frontu 25.

## Dáta v zdieľanej pamäti

Node vytvorí alebo zmení veľkosť mmap súboru na `mmap_path`. Súbor obsahuje:

- header s parametrom `LSYN`, verziou `1`, kapacitou, veľkosťou záznamu a `write_index`
- záznamy ring buffera s:
  - validity flags
  - LiDAR ROS header timestamp v nanosekundách
  - host receive timestamp v nanosekundách
  - lokálne poradové číslo LiDARu
  - `frame_id`

Ak existujúci mmap header nezodpovedá očakávanému formátu alebo nakonfigurovanej kapacite, súbor sa vymaže a znovu inicializuje.

## Overenie

Skontroluj, či prichádzajú LiDAR správy:

```bash
ros2 topic hz /lidar_points
```

Spusti bridge a sleduj jeho log. Každý prijatý point cloud by mal vytvoriť riadok s `lidar_seq`, `ros_header_ns`, `host_ns` a `frame_id`.

Môžeš tiež skontrolovať, či mmap súbor existuje:

```bash
ls -lh /dev/shm/lidar_stamp.bin
```
