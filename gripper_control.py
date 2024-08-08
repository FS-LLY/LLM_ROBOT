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
import signal
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import MyGripper
from omni.isaac.manipulators.grippers import ParallelGripper
import os

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)
assets_root_path = "/home/lly/.local/share/ov/pkg/isaac-sim-2023.1.1/extscache/omni.importer.urdf-1.6.1+105.1.lx64.r.cp310/data" #get_assets_root_path()
if assets_root_path is None:
    raise Exception("Could not find Isaac Sim assets folder")

asset_path = assets_root_path + "/urdf/robots/RIZON4/flexiv_rizon4_total/flexiv_rizon4_total.usd"# "/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd"
#print(asset_path)
#signal.pause()
add_reference_to_stage(usd_path=asset_path, prim_path="/World/rizon4")
# define the gripper
#print("here0")
gripper = MyGripper(
    # We chose the following values while inspecting the articulation
    end_effector_prim_path="/World/rizon4/base_hand",
    joint_prim_names=["left_outer_knuckle_joint", "right_outer_knuckle_joint","left_inner_finger_joint","right_inner_finger_joint"],#
    joint_opened_positions=np.array([0.6, 0.6,-0.6,-0.6]),
    joint_closed_positions=np.array([0.0, 0.0,0.0,0.0]),
    action_deltas=None,#np.array([0.28, 0.28,0.28,0.28]),
)
# define the manipulator
my_denso = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/rizon4",
        name="rizon4_robot",
        end_effector_prim_name="base_hand",
        gripper=gripper,
    )
)
# set the default positions of the other gripper joints to be opened so
# that its out of the way of the joints we want to control when gripping an object for instance.
joints_default_positions = np.zeros(14)
joints_default_positions[9] = 0.6
joints_default_positions[11] = 0.6
joints_default_positions[12] = -0.6
joints_default_positions[13] = -0.6
my_denso.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()
my_world.reset()
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
        
        gripper_positions = my_denso.gripper.get_joint_positions()
        if i < 500:
            # close the gripper slowly
            #my_denso.gripper.apply_action(
            #    ArticulationAction(joint_positions=[gripper_positions[0] - 0.01, gripper_positions[1] - 0.01,gripper_positions[2] + 0.01, gripper_positions[3] + 0.01])#
            #)
            #actions = ArticulationAction(joint_positions=[gripper_positions[0] - 0.01, gripper_positions[1] - 0.01,gripper_positions[2] + 0.01, gripper_positions[3] + 0.01])
            actions = target_joint_positions = my_denso.gripper.forward(action="close")
            my_denso.gripper.apply_action(actions)
        if i > 500:
            # open the gripper slowly
            #my_denso.gripper.apply_action(
            #    ArticulationAction(joint_positions=[gripper_positions[0] + 0.01, gripper_positions[1] + 0.01,gripper_positions[2] - 0.01, gripper_positions[3] - 0.01])#
            #)
            #actions = ArticulationAction(joint_positions=[gripper_positions[0] + 0.01, gripper_positions[1] + 0.01,gripper_positions[2] - 0.01, gripper_positions[3] - 0.01])
            actions = target_joint_positions = my_denso.gripper.forward(action="open")
            my_denso.gripper.apply_action(actions)
            
        if i == 1000:
            i = 0
        with open('./gripper_order.txt','a') as f:
            print(actions,file=f)
    if args.test is True:
        break

simulation_app.close()
