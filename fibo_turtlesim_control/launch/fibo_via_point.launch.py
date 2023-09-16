from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import os,yaml

def generate_launch_description():
    name = ['Foxy','Noetic','Humble','Tron'] # delcare turtle name
    index = 0

    turtleName = DeclareLaunchArgument('name',default_value='Got')

    create_path =  Node( # node for calling fibo_via_point_gen file for creating via_point
        package='fibo_turtlesim_control',
        executable='fibo_via_point_gen.py'
    )
    cmd = LaunchConfiguration(
        'cmd',
        default=['ros2 run turtlesim_plus turtlesim_plus_node.py']
    )
    turtlesim = ExecuteProcess( # run turtlesim_plus node for running turtlesim_plus_node.py
        cmd = [[cmd]],
        shell=True
    )

    cmd = LaunchConfiguration(
        'cmd',
        default=['ros2 service call /remove_turtle turtlesim/srv/Kill "{name: ','turtle1}"'] # call service for removing turtle
    )

    kill_turtle1 = ExecuteProcess( # run /remove_turtle
        cmd = [[cmd]],
        shell=True
    )

    standby=Node( # node for calling turtle_standby.py file
            package='fibo_turtlesim_control',
            executable='turtle_standby.py',
        )

    launch_description = LaunchDescription()
    launch_description.add_action(create_path)
    launch_description.add_action(standby)
    launch_description.add_action(turtlesim)
    launch_description.add_action(kill_turtle1)

    for _ in name: #เcall spawn_turtle service 
        cmd = LaunchConfiguration('cmd',default=['ros2 service call /spawn_turtle turtlesim/srv/Spawn "{x: ','1',', y: ','1',', theta: 0.0, name: \'',name[index],'\'}"'])    
        spawn_turtle = ExecuteProcess(
            cmd = [[cmd]],
            shell=True
        )
        
        controller=Node(# call Node controller.py
            package='fibo_turtlesim_control',
            namespace=name[index],
            executable='controller.py',
            remappings=[
                ('/pose','/'+name[index]+'/pose'),
                ('/cmd_vel','/'+name[index]+'/cmd_vel'),
                ]
        )
        scheduler=Node( # call Node scheduler.py
            package='fibo_turtlesim_control',
            executable='scheduler.py',
            namespace=name[index],
            parameters=[{'index':index}]
        )


        launch_description.add_action(spawn_turtle)
        launch_description.add_action(scheduler)
        launch_description.add_action(controller)
        index+=1 # increase index to go to next turtle
    
    return launch_description