import cv2
import depthai as dai
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

NODE_NAME = "multi_cam_node"
TARGET_MXID = "18443010A17F0C1300"

class MultiCamNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        all_devices = dai.Device.getAllAvailableDevices()
        self.device_info = None
        self.device = None  # Hold the dai.Device

        if not all_devices:
            self.get_logger().warn("No DepthAI devices found on startup!")
        else:
            self.get_logger().info(f"Found {len(all_devices)} DepthAI device(s):")
            for i, dev in enumerate(all_devices):
                self.get_logger().info(f"  • Device #{i+1}: MXID = {dev.getMxId()}")

        for dev in all_devices:
            if dev.getMxId() == TARGET_MXID:
                self.device_info = dev
                break

        if not self.device_info:
            self.get_logger().error(f"No OAK device found with MXID {TARGET_MXID}")
            raise RuntimeError(f"No OAK device found with MXID {TARGET_MXID}")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "camera/color/image_raw", 10)
        self.q_rgb = None

    def getPipeline(self, preview_res=(1448, 568)):
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(*preview_res)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def camera_initialization(self, debug=False, path="./"):
        # Build and start pipeline with the specified camera
        pipeline = self.getPipeline()
        usb2_mode = True  # or False, depending on your connection

        self.device = dai.Device(pipeline, self.device_info, usb2_mode)
        mxid = self.device.getMxId()
        self.get_logger().info(f"=== Connected to device: {mxid}")
        self.get_logger().info(f"   >>> Cameras: {[c.name for c in self.device.getConnectedCameras()]}"
)
        self.get_logger().info(f"   >>> USB speed: {self.device.getUsbSpeed().name}")

        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        if debug:
            self.image_display_opencv(path=path)
        else:
            while rclpy.ok():
                in_rgb = self.q_rgb.tryGet()
                if in_rgb is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(in_rgb.getCvFrame(), "bgr8")
                    self.publisher.publish(img_msg)
                if cv2.waitKey(1) == ord('q'):
                    break

    def image_display_opencv(self, path):
        img_cnt = 0
        while True:
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is not None:
                cv2.imshow("rgb", in_rgb.getCvFrame())
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    cv2.imwrite(os.path.join(path, f"{img_cnt}.bmp"), in_rgb.getCvFrame())
                    self.get_logger().info(f"Saved image: {img_cnt}")
                    img_cnt += 1
                elif key == ord('q'):
                    exit("User quit")

    def destroy_node(self):
        if self.device:
            self.get_logger().info("Closing DepthAI device")
            self.device.close()
            self.device = None
        return super().destroy_node()


def main():
    rclpy.init()
    cam_node = MultiCamNode()
    try:
        cam_node.camera_initialization(debug=False, path='/home/projects/sensor2_ws/src/camera/oakd_debug/cv_img_save')
        rclpy.spin(cam_node)
    except KeyboardInterrupt:
        pass
    finally:
        cam_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()