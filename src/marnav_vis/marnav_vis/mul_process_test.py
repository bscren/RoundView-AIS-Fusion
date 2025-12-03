import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from multiprocessing import Process, Queue, set_start_method
import os
from time import sleep
def worker(cam_idx, input_queue):
    print(f"Worker {cam_idx} started, pid={os.getpid()}")
    while True:
        img = input_queue.get()
        if img is None:
            print(f"Worker {cam_idx} exiting")
            break
        timestamp = rclpy.time.Time().to_msg()

        print(f"Worker {cam_idx} got image with shape: {img.shape}, in timestamp: {timestamp.sec}.{timestamp.nanosec}")

class MultiCamNode(Node):
    def __init__(self):
        super().__init__('multi_cam_node')
        self.bridge = CvBridge()
        self.cam_topics = ['/rtsp_image_0', '/rtsp_image_1', '/rtsp_image_2']
        self.queues = [Queue(maxsize=5) for _ in self.cam_topics]
        self.workers = [
            Process(target=worker, args=(i, self.queues[i]))
            for i in range(len(self.cam_topics))
        ]
        for p in self.workers:
            p.daemon = True
            p.start()
        self.subs = []
        for i, topic in enumerate(self.cam_topics):
            sub = self.create_subscription(
                Image, topic, lambda msg, idx=i: self.image_callback(msg, idx), 10
            )
            self.subs.append(sub)

    def image_callback(self, msg, idx):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not self.queues[idx].full():
            self.queues[idx].put(cv_img)
        else:
            self.get_logger().warn(f"Queue {idx} full, drop frame")

    def destroy_node(self):
        for q in self.queues:
            q.put(None)
        for p in self.workers:
            p.join(timeout=2)
        super().destroy_node()

def main(args=None):
    try:
        set_start_method('spawn')
    except RuntimeError:
        pass
    rclpy.init(args=args)
    node = MultiCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()