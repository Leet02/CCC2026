import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import sys

current_state = State()
current_pose = PoseStamped()

auto_arming = False

# 处理状态回调函数
def state_cb(msg):
    global current_state
    current_state = msg

# 处理本地位置回调函数
def local_position_cb(msg):
    global current_pose
    current_pose = msg

# 计算当前位置与目标位置之间的距离
def distance_to_target(target):
    dx = current_pose.pose.position.x - target[0]
    dy = current_pose.pose.position.y - target[1]
    dz = current_pose.pose.position.z - target[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # 订阅 MAVROS 状态和本地位置话题
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=local_position_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing 必须比 2Hz 更快
    rate = rospy.Rate(60)

    # 等待飞控连接
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()

    # 目标位置
    target1 = [0, 0, 1]
    target2 = [0, -3.5, 1]
    target3 = [6, -3.5, 1]
    target4 = [7.8, -3.5, 1]
    #target5 = [3, -1, 0.8]
    # target6 = [4, -1, 0.8]
    # target7 = [4.5, -1, 0.8]
    # target8 = [4.5, 1, 0.8]
    # target9 = [5.5, 1, 0.8]
    # target10 = [6.5, 0, 0.8]
    #targets = [target1,target2]
    targets = [target1]

    # 初始目标位置
    current_target = targets[0]
    pose.pose.position.x = current_target[0]
    pose.pose.position.y = current_target[1]
    pose.pose.position.z = current_target[2]

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    # while not rospy.is_shutdown():
    #     # 切换到 OFFBOARD 模式
    #     if current_state.mode != "OFFBOARD" :
    #         # 发送一些设定点
    #         for i in range(10):
    #             if rospy.is_shutdown():
    #                 break
    #             local_pos_pub.publish(pose)
    #             rate.sleep()
    #         if auto_arming and set_mode_client.call(offb_set_mode).mode_sent:
    #             rospy.loginfo("OFFBOARD 模式已启用")
    #     else:
    #         # 解锁飞行器
    #         if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
    #             if arming_client.call(arm_cmd).success:
    #                 rospy.loginfo("飞行器已解锁")
    #             last_req = rospy.Time.now()
    #     # if current_state.mode != "OFFBOARD" or not current_state.armed:
    #     #     rospy.loginfo("ready to takeoff...")
    #     #     continue
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
                for i in range(10):
                    if rospy.is_shutdown():
                        break
                    local_pos_pub.publish(pose)
                    rate.sleep()

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        # 发布当前设定点
        local_pos_pub.publish(pose)

        # 打印当前本地位置
        rospy.loginfo("当前本地位置: x = %.2f, y = %.2f, z = %.2f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)

        # 判断是否到达当前目标位置
        if distance_to_target(current_target) < 0.1:
            rospy.loginfo("已到达目标位置：%.2f, %.2f, %.2f", current_target[0], current_target[1], current_target[2])
            # 切换到下一个目标
            targets_index = targets.index(current_target)
            if targets_index < len(targets) - 1:
                current_target = targets[targets_index + 1]
                rospy.loginfo("切换到下一个目标：%.2f, %.2f, %.2f", current_target[0], current_target[1], current_target[2])
                # 更新目标位置
                pose.pose.position.x = current_target[0]
                pose.pose.position.y = current_target[1]
                pose.pose.position.z = current_target[2]
            else:
                rospy.loginfo("到达最终位置附近: x = %.2f, y = %.2f, z = %.2f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
            break
        rate.sleep()

    print("......................任务完成..............")
