import numpy as np
import time
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from src.Command import Command
from felin.Config import Configuration
from felin.Kinematics import four_legs_explicit_inverse_kinematics_BRF

# pat92fr
import asyncio
import math
import moteus
import moteus_pi3hat
# pat92fr

# pat92fr

# moteus IDs
servo_ids = np.array(
    [
        [ 1, 10, 4, 7],
        [ 2, 11, 5, 8],
        [ 3, 12, 6, 9]
    ]
)

leg_id_from_joint_id = {
    1:0,
    2:0,
    3:0,
    4:2,
    5:2,
    6:2,
    7:3,
    8:3,
    9:3,
    10:1,
    11:1,
    12:1
}

axis_id_from_joint_id = {
    1:0,
    2:1,
    3:2,
    4:0,
    5:1,
    6:2,
    7:0,
    8:1,
    9:2,
    10:0,
    11:1,
    12:2
}

# when the robot is sitting in the reference position, joint position offsets:
joint_position_offsets = np.zeros((3, 4))

# when seeing a moteus joint from the back, motor position is powitive turning CCW/TRIGO

# autocalibration is done is crouch position, so the angle offset is the IK of crouch feet positions
joint_angle_offsets = np.radians(np.array(
    [
        [   5.0,  -5.0,   5.0,  -5.0],
        [ 160.0, 160.0, 160.0, 160.0],
        [  24.4,  24.4,  24.4,  24.4]
    ]
))

joint_angle_directions = np.array(
    [
        [ 1.0, 1.0,-1.0,-1.0],
        [ 1.0,-1.0, 1.0,-1.0],
        [-1.0, 1.0,-1.0, 1.0]
    ]
)

# moteus :
#  one turn = +1
#  gear ratio = 1:6
#  angle = position * 2PI / 6
ratio = 2.0*math.pi / 6.0
joint_position_to_angle_ratios = np.array(
    [
        [ ratio, ratio, ratio, ratio],
        [ ratio, ratio, ratio, ratio],
        [ ratio, ratio, ratio, ratio]
    ]
)

def joint_angles_from_positions(positions):
    return np.multiply(np.multiply(positions-joint_position_offsets,joint_position_to_angle_ratios),joint_angle_directions)+joint_angle_offsets

def joint_positions_from_angles(angles):
    return np.divide(np.multiply(angles-joint_angle_offsets,joint_angle_directions),joint_position_to_angle_ratios)+joint_position_offsets
# pat92fr

async def main():
    global joint_position_offsets
    """Main program
    """
    # moteus
    # moteus
    # moteus
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1,2,3],
            2:[4,5,6],
            3:[7,8,9],
            4:[10,11,12]
        }
    )
    
    servos = {
        servo_id : moteus.Controller(id=servo_id, transport=transport)
        for servo_id in [1,2,3,4,5,6,7,8,9,10,11,12]
    }
    
    # We will start by sending a 'stop' to all servos, in the event
    # that any had a fault.
    await transport.cycle([x.make_stop() for x in servos.values()])
    
    # moteus
    # moteus
    # moteus
    
    # Create config
    print("Creating config...")
    config = Configuration()
    print("Done.")

    # create state
    print("Creating state...")
    state = State(config)
    print("Done.")
    
    # Create controller and user input handles
    print("Creating controller...")
    controller = Controller(config,four_legs_explicit_inverse_kinematics_BRF)
    print("Done.")
    
    # create joystick interface
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    # OVERRIDE JOINT MAX TORQUE config
    joint_max_torque = 4.0
    

    # autocalibration during  the first 1 seconde
    # store raw joint positions into positoin offsets
    print("Auto-calibration...")
    init_time = time.time()
    while True:
        
        # query position of joints
        commands = [
            servos[1].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[2].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[3].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[4].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[5].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[6].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[7].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[8].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[9].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[10].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[11].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True),
            servos[12].make_position(position=math.nan,velocity=0.0,maximum_torque=joint_max_torque,query=True)
        ]
        results = await transport.cycle(commands)
        # decode joint feedback
        temp_joint_present_position = np.zeros((3, 4))
        for result in results:
            temp_joint_present_position[
                axis_id_from_joint_id[result.id],
                leg_id_from_joint_id[result.id]
            ] = result.values[moteus.Register.POSITION]

        current_time = time.time()
        if current_time>init_time+1:
            joint_position_offsets = temp_joint_present_position
            print(joint_position_offsets)            
            break
        
        await asyncio.sleep(config.dt)
    print("Done.")     
    
    print("Passing to controller...")
    rate = 0.0
    counter = 0
    start_time = time.time()
    while True:

            
        # Read imu data. Orientation will be None if no data was available
        quat_orientation = np.array([1, 0, 0, 0])
        state.quat_orientation = quat_orientation

        # build command from joystick
        command = joystick_interface.get_command(state)
        #command = Command()

        # then run controller and compute new state
        controller.run(state, command)
        
        #print("state:"+str(state.behavior_state))
        #print("angle:\n"+str(np.round(np.degrees(state.joint_angles),0)))
        #print("position:\n"+str(np.round(joint_positions_from_angles(state.joint_angles),3)))
        # Hardware Interface, realloc servo matrix to servo ID
        temp_joint_present_position = np.zeros((3, 4))
        temp_joint_present_torque = np.zeros((3, 4))

        if True:
            temp_joint_positions = joint_positions_from_angles(state.joint_angles)
            commands = [
                servos[1].make_position(
                    position=temp_joint_positions[0,0],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[2].make_position(
                    position=temp_joint_positions[1,0],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[3].make_position(
                    position=temp_joint_positions[2,0],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),

                servos[4].make_position(
                    position=temp_joint_positions[0,2],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[5].make_position(
                    position=temp_joint_positions[1,2],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[6].make_position(
                    position=temp_joint_positions[2,2],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),

                servos[7].make_position(
                    position=temp_joint_positions[0,3],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[8].make_position(
                    position=temp_joint_positions[1,3],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[9].make_position(
                    position=temp_joint_positions[2,3],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                
                servos[10].make_position(
                    position=temp_joint_positions[0,1],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[11].make_position(
                    position=temp_joint_positions[1,1],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                ),
                servos[12].make_position(
                    position=temp_joint_positions[2,1],
                    velocity=0.0,
                    kp_scale=4,
                    maximum_torque=joint_max_torque,
                    query=True
                )
                
            ]
            
            results = await transport.cycle(commands)
            
            # decode joint feedback

            for result in results:
                temp_joint_present_position[
                    axis_id_from_joint_id[result.id],
                    leg_id_from_joint_id[result.id]
                ] = result.values[moteus.Register.POSITION]
            for result in results:
                temp_joint_present_torque[
                    axis_id_from_joint_id[result.id],
                    leg_id_from_joint_id[result.id]
                ] = result.values[moteus.Register.TORQUE]
            
        # stats
        counter += 1
        if counter%100 == 0:
            rate = 0.5*rate + 0.5*counter/(time.time()-start_time)
            print("rate:"+str(round(rate,1))+"Hz")
            print("torque:"+str(temp_joint_present_torque[2,2])+"Nm")


            # update state feedback
#             # FR
#             joint_position_offsets[0,0] = temp_joint_present_position[0,0]
#             joint_position_offsets[1,0] = temp_joint_present_position[1,0]
#             joint_position_offsets[2,0] = temp_joint_present_position[2,0]
#             # FL
#             joint_position_offsets[0,1] = temp_joint_present_position[0,3]
#             joint_position_offsets[1,1] = temp_joint_present_position[1,3]
#             joint_position_offsets[2,1] = temp_joint_present_position[2,3]
#             # RR
#             joint_position_offsets[0,2] = temp_joint_present_position[0,1]
#             joint_position_offsets[1,2] = temp_joint_present_position[1,1]
#             joint_position_offsets[2,2] = temp_joint_present_position[2,1]
#             # RL
#             joint_position_offsets[0,3] = temp_joint_present_position[0,2]
#             joint_position_offsets[1,3] = temp_joint_present_position[1,2]
#             joint_position_offsets[2,3] = temp_joint_present_position[2,2]            
# #             
#             print(", ".join(
#                 f"({result.arbitration_id} " +
#                 f"{result.values[moteus.Register.POSITION]} " +
#                 f"{result.values[moteus.Register.TORQUE]})"
#                 for result in results))

        #await asyncio.sleep(0.0005)
        #await asyncio.sleep(config.dt)
        #await asyncio.sleep(1)
        
if __name__ == '__main__':
    asyncio.run(main())
