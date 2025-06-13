import cv2
# from roboflowoak import RoboflowOak
import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

NODE_NAME = 'balloon_node'

class BalloonNode(Node):
	def __init__(self):
		super().__init__('balloon_node')
		self.publisher = self.create_publisher(String, 'balloon_detections', 1)

		# Path to your .blob file
		self.model_path = "/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_balloon2_pkg/best_openvino_2022.1_6shave.blob"

		# Class labels (in order from your training)
		self.class_names = ["dog", "fish", "snake", "swan", "turtle"]  # adjust to match your training set

		# Set up pipeline
		self.pipeline = dai.Pipeline()

		cam_rgb = self.pipeline.create(dai.node.ColorCamera)
		cam_rgb.setPreviewSize(640, 640)
		cam_rgb.setInterleaved(False)
		cam_rgb.setFps(25)

		detection_nn = self.pipeline.create(dai.node.YoloDetectionNetwork)
		detection_nn.setBlobPath(self.model_path)
		detection_nn.setConfidenceThreshold(0.5)
		detection_nn.setNumClasses(len(self.class_names))
		detection_nn.setCoordinateSize(4)
		detection_nn.setIouThreshold(0.4)
		#detection_nn.setInputSize(640, 640)
		detection_nn.input.setBlocking(False)
		detection_nn.input.setQueueSize(1)

		cam_rgb.preview.link(detection_nn.input)

		xout_rgb = self.pipeline.create(dai.node.XLinkOut)
		xout_rgb.setStreamName("rgb")
		cam_rgb.preview.link(xout_rgb.input)

		xout_nn = self.pipeline.create(dai.node.XLinkOut)
		xout_nn.setStreamName("detections")
		detection_nn.out.link(xout_nn.input)

		device_id = self.declare_parameter('device_name', '').get_parameter_value().string_value
		devs = dai.Device.getAllAvailableDevices()
		if not devs:
			self.get_logger().warn("No DepthAI devices found!")
		else:
      			for i, dev in enumerate(devs):
                        self.get_logger().info(f"DepthAI device #{i+1}: MXID = {dev.getMxId()}")
        selected_device = None
        for dev in devs:
			if dev.getMxId() == device_id:
				selected_device = dev
				break

		if selected_device:
			self.get_logger().info(f"Using OAK-D with MxId={selected_device.getMxId()}")
			self.device = dai.Device(self.pipeline, selected_device)
		else:
			self.get_logger().warn(f"OAK-D with MxId={device_id} not found. Defaulting to first available.")
			self.device = dai.Device(self.pipeline)

		self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
		self.q_dets = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)

		self.timer = self.create_timer(0.05, self.detect_and_publish)

		self.start = time.time()

	def detect_and_publish(self):
		# Run the Roboflow model
		# instantiating an object (rf) with the RoboflowOak module
		#running our model and displaying the video output with detection
		#the rf.detect() function runs the model inference
	
		in_rgb = self.q_rgb.tryGet()
		in_nn = self.q_dets.tryGet()

		if in_rgb is None or in_nn is None:
			return

		frame = in_rgb.getCvFrame()
		detections = in_nn.detections

		balloon_type = 'none'
		for det in detections:
			class_id = det.label
			if 0 <= class_id < len(self.class_names):
				balloon_type = self.class_names[class_id]
				break

		elapsed = time.time() - self.start
		msg = String()
		msg.data = f"Detected: {balloon_type} at {elapsed:.2f}"
		self.publisher.publish(msg)
		self.get_logger().info(f"Published: {msg.data}")
	
		

def main(args=None):
	#time.sleep(5)
	rclpy.init(args=args)
	balloon_publisher = BalloonNode()
	try:
		rclpy.spin(balloon_publisher)
	except KeyboardInterrupt:
		balloon_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
	finally:
		balloon_publisher.destroy_node()
		rclpy.shutdown()
		balloon_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ ==  '__main__':
	main()