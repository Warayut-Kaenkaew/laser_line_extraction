from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import lifecycle_msgs.msg


def generate_launch_description():
    ld =LaunchDescription()
    laser_dir = get_package_share_directory("laser_line_extraction") 
    config_path = os.path.join(laser_dir,"config","laser_line_extraction.yaml")
    params_file = LaunchConfiguration('params_file')
    auto_activate = LaunchConfiguration('auto_activate')

    declarre_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=config_path,
        description="Full path to param file"
    )

    declare_auto_activate_cmd = DeclareLaunchArgument(
        'auto_activate', 
        default_value='true', 
        description='set true, the node will automatically set to active state'
    )

    print(f"Using parameters file: {config_path}")

    laser_line_extrction_lc = LifecycleNode(
        namespace="",
        name="laser_line_extraction_lifecycle",
        package="laser_line_extraction",
        executable="line_extraction_lc",
        parameters=[params_file],
    )


    #transit to configured state
    on_configure_trans = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(laser_line_extrction_lc),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        
    )

    #transit to active state
    on_activate_trans = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(laser_line_extrction_lc),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ),
        
    )
    
    #checkout if auto activate
    activate_node = GroupAction(
        condition=IfCondition(auto_activate),
        actions=[
            on_configure_trans,
            on_activate_trans,
        ],
    )

    #shutdown event
    shutdown = EmitEvent(
        event=Shutdown(
            reason="shutdown() was called"
        )
    )

    #unconfigured state handler -> onConfigured transition
    unconfigured_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=laser_line_extrction_lc,
            goal_state='unconfigured', 
            entities= [
                on_configure_trans,
                LogInfo( msg = "'laser line extraction node' is in the 'UNCONFIGURED' state" ),
            ],

        )
    )
    
    #finalized state handler -> end program
    finalized_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=laser_line_extrction_lc,
            goal_state='finalized', 
            entities= [
                shutdown,
                LogInfo( msg = "shut down greatly" ),
            ],

        )
    )

    ld.add_action(declarre_params_file_cmd)
    ld.add_action(declare_auto_activate_cmd)

    ld.add_action(laser_line_extrction_lc)
    ld.add_action(activate_node)

    ld.add_action(unconfigured_state_handler)
    ld.add_action(finalized_state_handler)
    return ld