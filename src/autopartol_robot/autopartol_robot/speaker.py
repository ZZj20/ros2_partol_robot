import rclpy
from rclpy.node import Node
from autopatol_interfaces.srv import SpeechText
import espeakng

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.speech_service_=self.create_service(SpeechText,'speech_text',self.speech_text_callback)
        self.speaker_ =espeakng.Speaker()
        self.speaker_.voice='zh'

    def speech_text_callback(self,request,response):
        self.get_logger().info(f'正准备朗读{request.text}')
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result=True
        return response
    


def main():
    rclpy.init()
    node =Speaker()
    rclpy.spin(node)
    rclpy.shutdown()
    