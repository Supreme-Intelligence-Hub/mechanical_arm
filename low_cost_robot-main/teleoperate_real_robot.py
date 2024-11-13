from robot import Robot
from dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbmodem57380045631').instantiate()
follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbserial-FT8ISNO8').instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 6, 7])

# **设置触发扭矩**：
#- `leader.set_trigger_torque()`：在`leader`实例上设置触发扭矩，通常是为链条中的最后一个舵机设置固定扭矩，以便主机系统能够带动从机系统。
leader.set_trigger_torque()


while True:
    # 在每次循环中，`follower`会读取`leader`的当前位置，并通过`set_goal_pos`将其设置为自己的目标位置。
    follower.set_goal_pos(leader.read_position())
