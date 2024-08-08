# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp
from data_generator import send_func, isaac2data

simulation_app = SimulationApp({"headless": True})

import argparse

import numpy as np
from controllers.pick_place import PickPlaceController
from omni.isaac.core import World
from tasks.pick_place import PickPlace
import time, os
import subprocess
import json

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test", default=False, action="store_true", help="Run in test mode"
)
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)

# 替换为你的C++程序的路径
# cpp_program_path = "cmake-build-debug/base/test_every_joint"
cpp_program_path = "standalone_examples/api/omni.isaac.manipulators/RIZON4/cmake-build-debug/base/test_every_joint"

# 启动C++程序并创建一个到其标准输入的管道
p = subprocess.Popen(cpp_program_path, stdin=subprocess.PIPE)

# data need to filled up by LLM
# target_position = np.array([0.4, -0.5, 0])
target_position = np.array([1, -0.5, 0])
target_position[2] = 0.0515 / 2.0
cube_position = np.array([0.6, -0.4, 0])
cube_position[2] = 0.0515 / 2.0
my_task = PickPlace(
    name="denso_pick_place",
    cube_initial_position=cube_position,
    target_position=target_position,
    cube_initial_orientation=None,
    cube_size=np.array([0.033, 0.033, 0.038]),
)

my_world.add_task(my_task)
my_world.reset()

my_denso = my_world.scene.get_object("rizon4_robot")
# initialize the controller
my_controller = PickPlaceController(
    name="controller", robot_articulation=my_denso, gripper=my_denso.gripper
)
task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()
i = 0
last_timestamp = time.time()
this_timestamp = 0
last_pos = [0, -40.0 * np.pi / 180.0, 0, 90 * np.pi / 180, 0, 40 / 180 * np.pi, 0]

while simulation_app.is_running():
    i = i + 1
    my_world.step(render=False)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        # forward the observation values to the controller to get the actions
        # print(observations[task_params["cube_name"]["value"]]["position"])
        # input()

        actions = my_controller.forward(  ##todo
            picking_position=observations[task_params["cube_name"]["value"]][
                "position"
            ],
            placing_position=observations[task_params["cube_name"]["value"]][
                "target_position"
            ],
            current_joint_positions=observations[task_params["robot_name"]["value"]][
                "joint_positions"
            ],
            end_effector_offset=np.array([0.004, 0, 0.18]),
            # end_effector_orientation = np.array([0,0.7071,0.7071,0]),
        )
        """
        actions = my_controller.forward(##todo
            picking_position=np.array([0.5, 0.5, 0.2]),
            placing_position=np.array([0.5, 0, 0.2]),
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0.0, 0, 0.2]),
            #end_effector_orientation = np.array([0,0.707,0.707,0]),
        )
        """
        with open("./order.txt", "a") as f:
            print(actions, file=f)
        this_timestamp = time.time()
        # input()
        time_span = this_timestamp - last_timestamp
        last_timestamp = this_timestamp
        # print(int(time_span*1000))
        send_timespan = int(time_span * 1000)
        # print(send_timespan)
        data = isaac2data(actions)
        # print(data['joint_positions'])
        send_func(time_span=send_timespan, action=str(actions),process=p,last_pos=last_pos)
        
        last_pos = data['joint_positions']
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
    if args.test is True:
        break

simulation_app.close()
