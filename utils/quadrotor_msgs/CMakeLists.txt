cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        geometry_msgs
        message_generation
)

add_message_files(
        FILES
        PositionCommand.msg
        MpcPositionCommand.msg
        PolynomialTrajectory.msg
        TrakingPerformance.msg
        QuadrotorState.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        nav_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)
