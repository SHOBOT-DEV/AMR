def register_ros2_routes(bp, store):
    from .Packages import register_ros2_package_routes
    from .Robot import register_ros2_robot_routes
    from .Sensors import register_ros2_sensor_routes
    from .Safety import register_ros2_safety_routes
    from .Dock import register_ros2_dock_routes
    from .Params import register_ros2_param_routes
    from .Navigation import register_ros2_navigation_routes
    from .RosGraph import register_ros_graph_routes

    register_ros2_robot_routes(bp, store)
    register_ros2_sensor_routes(bp, store)
    register_ros2_safety_routes(bp, store)
    register_ros2_dock_routes(bp, store)
    register_ros2_param_routes(bp, store)
    register_ros2_navigation_routes(bp, store)
    register_ros2_package_routes(bp, store)
    register_ros_graph_routes(bp, store)


__all__ = ["register_ros2_routes"]
