# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import numpy as np
from controllers.rmpflow import RMPFlowController
from omni.isaac.core import World
from tasks.follow_target import FollowTarget

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
# Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="denso_follow_target", target_position=np.array([0.6807, -0.119, 0.2859]))#np.array([0.6807, -0.119, 0.2859])
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("denso_follow_target").get_params()
target_name = task_params["target_name"]["value"]
print(target_name)
print(my_task._target_prim_path)
denso_name = task_params["robot_name"]["value"]
my_denso = my_world.scene.get_object(denso_name)

# initialize the controller
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_denso)

# make RmpFlow aware of the ground plane
ground_plane = my_world.scene.get_object(name="default_ground_plane")
my_controller.add_obstacle(ground_plane)

i = 0
articulation_controller = my_denso.get_articulation_controller()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"]+np.array([0.0,0.001,0])*i,
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        #my_task.set_params(target_prim_path= my_task._target_prim_path,target_name=my_task._target_name, target_position = observations[target_name]["position"]+np.array([0,0,0.001]))
        with open('./order.txt','a') as f:
            print(actions,file=f)
        articulation_controller.apply_action(actions)
        i = i+1
    if i > 300: 
        break

    if args.test is True:
        break
simulation_app.close()
