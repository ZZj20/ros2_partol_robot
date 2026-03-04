import rclpy
from rclpy.node import Node 
import rclpy.time
from tf2_ros import TransformListener , Buffer #坐标监听器
from tf_transformations import euler_from_quaternion #四元数转欧拉角
import math #角度转弧度


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.buffer =Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.timer_=self.create_timer(1,self.get_transform)

    def get_transform(self):
        #实时获取坐标关系
        try:
            result =self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time(seconds=0),rclpy.time.Duration(seconds=1.0))
            transform=result.transform
            self.get_logger().info(f'平移：{transform.translation}')
            self.get_logger().info(f'旋转：{transform.rotation}')
            rotation_euler =euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w]
            )
            self.get_logger().info(f'旋转RPY：{rotation_euler}')


        except Exception as e:
            self.get_logger().warn(f"获取坐标关系失败，原因是：{str(e)}")



def main():
    rclpy.init()
    node =TFListener()
    rclpy.spin(node)
    rclpy.shutdown()

