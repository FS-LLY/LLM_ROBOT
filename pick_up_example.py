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
from controllers.pick_place import PickPlaceController
from omni.isaac.core import World
from tasks.pick_place import PickPlace
import time,os

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

my_world = World(stage_units_in_meters=1.0)

#data need to filled up by LLM
# target_position = np.array([0.4, -0.5, 0])
#assign the target
target_position = [np.array([0.4,-0.25,0]),np.array([0.5,-0.3,0])]
target_position[0][2] =target_position[0][2]+ 0.038 / 2.0
target_position[1][2] =target_position[1][2]+ 0.038 / 2.0
#assign the cube position
cube_position = [np.array([0.5, 0, 0.2]),np.array([0.3, -0.4, 0])]
cube_position[0][2] = cube_position[0][2]+ 0.038 / 2.0
cube_position[1][2] = cube_position[1][2]+ 0.038 / 2.0
cube_size=[np.array([0.033, 0.033, 0.038]),np.array([0.033, 0.033, 0.038])]
#assign the number of object 
my_task = PickPlace(name="denso_pick_place",kind="cube",obj_num = 2,cube_initial_position= cube_position,target_position=target_position,cube_initial_orientation  = None,cube_size=cube_size)
my_world.add_task(my_task)
my_world.reset()

my_denso = my_world.scene.get_object("rizon4_robot")
# initialize the controller
my_controller = PickPlaceController(name="controller", robot_articulation=my_denso, gripper=my_denso.gripper)
task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()
i = 0
action = -1
start_time = time.time()
while simulation_app.is_running():
    my_world.step(render=True)#Time cost
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        # forward the observation values to the controller to get the actions
        #print(observations[task_params["cube_name"]["value"]]["position"])
        #input()
        # print(observations[task_params["cube_name"]["value"]])


        actions = my_controller.forward(##todo
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0.0, 0, 0.18]),
            #end_effector_orientation = np.array([0,0.7071,0.7071,0]),
        )

        '''
        actions = my_controller.forward(##todo
            picking_position=np.array([0.5, 0.5, 0.2]),
            placing_position=np.array([0.5, 0, 0.2]),
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0.0, 0, 0.2]),
            #end_effector_orientation = np.array([0,0.707,0.707,0]),
        )
        
        '''
        if action<my_controller._event:
            end_time = time.time()
            print("action ",str(action), " time:",(end_time-start_time)*1000," ms")
            action = my_controller._event
            start_time = time.time()
        num = my_task.task_num
        text = "./standalone_examples/api/omni.isaac.manipulators/RIZON4/simulate_datasets/order" + str(num) + ".txt"
        #text = "./simulate_datasets/order" + str(num) + ".txt"

        with open(text, 'a', encoding='utf-8') as file:
            local_action = str(actions)
            local_action = local_action.replace("'",'"')
            file.write(local_action + "\n")

        #with open(text,'a') as f:
            #f.write(actions)
            #print(actions,file=f)

        #input()
        if my_controller.is_done():
            print("done picking and placing")
            my_controller.reset()
            my_task.task_num += 1
            if my_task.task_num >=my_task._obj_num:
                exit()
            print(my_task.task_num)
            task_params = my_world.get_task("denso_pick_place").get_params()
            action = -1
            
        articulation_controller.apply_action(actions)
        #input()

    if args.test is True:
        break

simulation_app.close()

