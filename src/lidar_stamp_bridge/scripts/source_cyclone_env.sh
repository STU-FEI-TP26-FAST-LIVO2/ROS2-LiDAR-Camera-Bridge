#!/bin/bash
# source_cyclone_env.sh  –  nastav Cyclone DDS pred rucnym spustenim nodu
#
# Pouzitie (source, nie bash!):
#   source scripts/source_cyclone_env.sh
#   ros2 run lidar_stamp_bridge lidar_stamp_bridge_node
#
# Alebo pre launch:
#   source scripts/source_cyclone_env.sh
#   ros2 launch lidar_stamp_bridge lidar_stamp_bridge.launch.py

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${SCRIPT_DIR}/../config/cyclonedds.xml"

echo "Cyclone DDS nastavene:"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI}"
