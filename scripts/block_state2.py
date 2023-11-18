#!/usr/bin/env python3
from re import S
import rospy
import moveit_commander
import actionlib
import math
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

gazebo_model_states = ModelStates()

def callback(msg):
    global gazebo_model_states
    gazebo_model_states = msg

def yaw_of(object_orientation):
    euler = euler_from_quaternion(
        (object_orientation.x, object_orientation.y,
        object_orientation.z, object_orientation.w))
    return euler[2]

def main():
    global gazebo_model_states

    OBJECT_NAME = "Block_blue_50mm"
    GRIPPER_OPEN = 0.5
    GRIPPER_CLOSE = 0.23
    APPROACH_Z = 0.12
    LEAVE_Z = 0.20
    PICK_Z = 0.1
    TP = Point(0.025,-0.375,0)#ターゲットポジション
    PP = Point(TP.x+0.1,TP.y+0.1,0.5)#ターゲットポジションからx,y方向で0.1を足したもの
    SMP = Point(PP.x+0.05,PP.y,0.12)#直接経路衝突回避のための経由地点
    SP = Point(SMP.x,PP.y,0.05)#ストッパに向けてアームを動かすときのスタート地点x
    YSX = Point(SMP.x-0.166,PP.y,0.05)#ブロックをx方向に向かって引きずるプログラム
    SP2 = Point(PP.x-0.093,PP.y + 0.1,0.1)#ストッパに向けてアームを動かすときのスタート地点y
    YSY = Point(SP2.x,SP2.y -0.18,0.1)#ブロックをy方向に向かって引きずるプログラム
   
 
    sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(0.6)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    rospy.sleep(1.0)

    # 何かを掴んでいた時のためにハンドを開く
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1.0)

    # オブジェクトがgazebo上に存在すれば、pick_and_placeを実行する
    if OBJECT_NAME in gazebo_model_states.name:
        object_index = gazebo_model_states.name.index(OBJECT_NAME)
        object_position = gazebo_model_states.pose[object_index].position
        object_orientation = gazebo_model_states.pose[object_index].orientation
        object_yaw = yaw_of(object_orientation)

        # オブジェクトに接近する
        target_pose = Pose()
        target_pose.position.x = object_position.x
        target_pose.position.y = object_position.y
        target_pose.position.z = APPROACH_Z
        q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # 掴みに行く
        target_pose.position.z = PICK_Z
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        gripper_goal.command.position = GRIPPER_CLOSE
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

        # 持ち上げる
        target_pose.position.z = LEAVE_Z
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # 設置位置に移動する
        target_pose.position = PP
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # 設置する
        target_pose.position.z = PICK_Z
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        gripper_goal.command.position = GRIPPER_OPEN
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

        # ハンドを上げる
        target_pose.position.z = LEAVE_Z
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        
        # ハンドを閉じる
        gripper_goal.command.position = 0
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(1.0))

        # 設置位置に移動する
        target_pose.position = SMP
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # 設置位置に移動する
        target_pose.position = SP
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # 設置位置に移動する
        target_pose.position = YSX
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()
        rospy.sleep(1.0)

        # 設置位置に移動する
        target_pose.position = SP2
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)

        # 設置位置に移動する
        target_pose.position = YSY
        q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        


        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()
        rospy.sleep(1.0)


        print("Done")
    else:
        print("No objects")

if __name__ == '__main__':
    rospy.init_node("pick_and_place_in_gazebo_example")
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

