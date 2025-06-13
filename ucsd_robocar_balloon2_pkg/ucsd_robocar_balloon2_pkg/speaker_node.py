import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import subprocess

NODE_NAME = 'speaker_node'

class SpeakerNode(Node):
	def __init__(self):
		super().__init__(NODE_NAME)
		self.subscription = self.create_subscription(
			String,
			'balloon_detections',
			self.listener_callback,
			1
		)
		self.lastPlayed = 0
		self.cooldown = 5
		#'/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/audio/dogbarking.wav'

		self.sounds = {
			'dog': '/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/audio/Dog.wav',
			'turtle': '/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/audio/Turtle.wav',
			'fish': '/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/audio/Fish.wav',
			'swan': '/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/audio/Swan.wav',
			'snake': '/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/audio/Snake.wav',
		}


	def listener_callback(self, msg):
		now = time.time()
		for animal, sound_path in self.sounds.items():
			if animal in msg.data and (now - self.lastPlayed) > self.cooldown:
				self.get_logger().info(f"[SPEAKER] Detected a {animal} balloon! Playing sound...")
				self.play_sound(sound_path)
				self.lastPlayed = now
				break

	def play_sound(self, sound_path):
		try: 
			subprocess.run(
				['aplay', '-D', 'plughw:2,0', sound_path],
				check=True
			)
		except subprocess.CalledProcessError as e:
			self.get_logger().error(f"Failed to play audio: {e}")

def main(args=None):
    rclpy.init(args=args)
    speaker_node = SpeakerNode()
    try:
        rclpy.spin(speaker_node)
    except KeyboardInterrupt:
        speaker_node.get_logger().info(f'Shutting down {NODE_NAME}...')
    finally:
        speaker_node.destroy_node()
        rclpy.shutdown()
        speaker_node.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()