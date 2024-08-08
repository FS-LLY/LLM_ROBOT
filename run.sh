#!/bin/bash

rm -f /home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/simulate_datasets/order*

# 运行 path/chat_pick_up.py
python3 /home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/chat_pick_up.py
if [ $? -ne 0 ]; then
    echo "Error: Failed to run chat_pick_up.py"
    exit 1
fi

# 运行启动环境的脚本
/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh /home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/standalone_examples/api/omni.isaac.manipulators/RIZON4/generated_code.py
if [ $? -ne 0 ]; then
    echo "Error: Failed to run python.sh"
    exit 1
fi



echo "All scripts ran successfully."
