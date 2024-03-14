from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
import lifecycle_msgs.msg
import launch
from launch import LaunchIntrospector
from launch.events import matches_action

def generate_launch_description():
    laser_dir = get_package_share_directory("laser_line_extraction") 
    config_path = os.path.join(laser_dir,"config","laser_line_extraction.yaml")
    params_file = LaunchConfiguration('params_file')

    declarre_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=config_path,
        description="Full path to param file"
    )

    print(f"Using parameters file: {config_path}")
    ld = LaunchDescription()

    laser_line_extrction_lc = LifecycleNode(
        namespace="ouranos",
        name="laser_line_extraction_lifecycle",
        package="laser_line_extraction",
        executable="line_extraction_lc",
        parameters=[params_file]
    )

    #transit to configured state
    #lifecycle_node_matcher is a callable which returns True if the
    #given lifecycle node should be affected by this event.
    lc_on_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(laser_line_extrction_lc),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    
    #unconfigured state handler -> onConfigured transition
    lc_unconfigured_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=laser_line_extrction_lc,
            goal_state='unconfigured', # you will do action in entity when the node is in this state
            entities= [
                lc_on_configure_trans_event,
                LogInfo( msg = "'laser line extraction node' is in the 'INACTIVE' state" ),
            ],

        )
    )
    


    ld.add_action(declarre_params_file_cmd)
    ld.add_action(lc_unconfigured_state_handler)
    ld.add_action(laser_line_extrction_lc)
    ld.add_action(lc_on_configure_trans_event)
    return ld