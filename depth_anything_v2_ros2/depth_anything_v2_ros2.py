#!/usr/bin/env python

# Copyright (c) 2024 Óscar Pons Fernández <oscarpf22@gmail.com>
# Copyright (c) 2024 Alberto J. Tudela Roldán <ajtudela@gmail.com>
# Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Python
import copy
import time

import torch
from cv_bridge import CvBridge, CvBridgeError

# ROS 2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# DepthAnything V2
from depth_anything_v2_ros2.depth_estimator import DepthEstimator, resolve_device

SENSOR_QOS_PROFILE = QoSProfile(
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1)


class DepthAnythingROS(LifecycleNode):
    """DepthAnythingROS lifecycle node.

    This managed node subscribes to an RGB image topic and publishes its
    metric depth estimation. The model is loaded in ``on_configure`` and
    the topics are created in ``on_activate``, so a misconfigured encoder
    or a missing checkpoint results in a clean configure failure instead
    of crashing the process.

    Parameters
    ----------
        image_topic : str
            Topic where the RGB image will be subscribed.
        camera_info_topic : str
            Topic where the camera info matching `image_topic` will be subscribed.
        depth_image_topic : str
            Topic where the depth image will be published.
        depth_camera_info_topic : str
            Topic where the camera info matching the depth image will be published.
        device : str
            Device to use for the inference (cpu or cuda).
        model_file : str
            Path to the model.
        encoder : str
            Encoder to use for the model (vits, vitb or vitl).
        max_depth : float
            Maximum depth, in meters, the checkpoint was trained with.

    Subscribers
    ----------
        image_topic : sensor_msgs.msg.Image
            RGB image to estimate the depth from.
        camera_info_topic : sensor_msgs.msg.CameraInfo
            Camera info matching `image_topic`, republished alongside the depth image.

    Publishers
    ----------
        depth_image_topic : sensor_msgs.msg.Image
            Estimated depth image, in 32FC1.
        depth_camera_info_topic : sensor_msgs.msg.CameraInfo
            Camera info matching the published depth image.
    """

    def __init__(self):
        super().__init__('depth_anything')
        self.bridge = CvBridge()
        self.estimator = None
        self.image_sub = None
        self.camera_info_sub = None
        self.depth_image_pub = None
        self.depth_camera_info_pub = None
        self.last_camera_info = None
        # Inference is heavy (tens/hundreds of ms); isolating it in its own
        # callback group lets a MultiThreadedExecutor keep servicing other
        # callbacks (e.g. lifecycle transitions) while it runs.
        self.inference_callback_group = MutuallyExclusiveCallbackGroup()

        self.get_params()

    def _declare(self, name, default):
        """Declare a parameter, read its resolved value and log it.

        Parameters
        ----------
        name : str
            Parameter name.
        default : Any
            Default value for the parameter.

        Returns
        -------
        Any
            The resolved parameter value.
        """
        value = self.declare_parameter(name, default).value
        self.get_logger().info(f'Parameter [{name}] set to: [{value}]')
        return value

    def get_params(self) -> None:
        """Declare, read and validate the node parameters.

        Raises
        ------
        ValueError
            If `max_depth` is not strictly positive.
        """
        self.image_topic = self._declare(
            'image_topic', 'camera/color/image_raw')
        self.camera_info_topic = self._declare(
            'camera_info_topic', 'camera/color/camera_info')
        self.depth_image_topic = self._declare('depth_image_topic', 'depth')
        self.depth_camera_info_topic = self._declare(
            'depth_camera_info_topic', 'depth/camera_info')
        self.device = self._declare('device', 'cuda:0')
        self.model_file = self._declare(
            'model_file', 'depth_anything_v2_vits.pth')
        self.encoder = self._declare('encoder', 'vits')
        self.max_depth = self._declare('max_depth', 10.0)

        if self.max_depth <= 0:
            raise ValueError(
                f'Wrong max_depth: [{self.max_depth}]. Must be greater than 0')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load the depth estimation model and create the lifecycle publishers.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if the model was loaded, FAILURE otherwise.
        """
        self.device = resolve_device(self.device, torch.cuda.is_available())
        self.get_logger().info(f'Setting device to: [{self.device}]')

        self.estimator = DepthEstimator(
            self.encoder, self.model_file, self.device, self.max_depth)
        try:
            self.estimator.load()
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(
                f'Failed to load the depth estimation model: {e}')
            return TransitionCallbackReturn.FAILURE

        self.depth_image_pub = self.create_lifecycle_publisher(
            Image, self.depth_image_topic, SENSOR_QOS_PROFILE)
        self.depth_camera_info_pub = self.create_lifecycle_publisher(
            CameraInfo, self.depth_camera_info_topic, SENSOR_QOS_PROFILE)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create the subscribers and activate the lifecycle publishers."""
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, SENSOR_QOS_PROFILE,
            callback_group=self.inference_callback_group)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, SENSOR_QOS_PROFILE)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Destroy the subscribers and deactivate the lifecycle publishers."""
        self.destroy_subscription(self.image_sub)
        self.destroy_subscription(self.camera_info_sub)
        self.image_sub = None
        self.camera_info_sub = None
        self.last_camera_info = None

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Release the model and the lifecycle publishers."""
        self.destroy_lifecycle_publisher(self.depth_image_pub)
        self.destroy_lifecycle_publisher(self.depth_camera_info_pub)
        self.depth_image_pub = None
        self.depth_camera_info_pub = None
        self.estimator = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handle shutdown requested from any state."""
        return TransitionCallbackReturn.SUCCESS

    def camera_info_callback(self, camera_info_msg: CameraInfo) -> None:
        """Store the latest camera info to republish it alongside the depth image."""
        self.last_camera_info = camera_info_msg

    def image_callback(self, image_msg: Image) -> None:
        """Estimate and publish the metric depth of the incoming image.

        Args:
            image_msg (Image): Image message.
        """
        # Only run inference if there's any subscriber.
        if self.depth_image_pub.get_subscription_count() == 0:
            return

        self.get_logger().info(
            f'Subscribed to depth image topic: [{self.depth_image_topic}]', once=True)
        start_time = time.time()

        # Convert ROS Image to OpenCV Image
        try:
            current_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(
                f'Could not convert depth image to OpenCV: {e}')
            return

        # Perform inference and clip the result to max depth
        depth = self.estimator.infer(current_image)

        execution_time = time.time() - start_time
        self.get_logger().debug(
            f'Depth estimation took: {execution_time * 1000:.1f} ms')

        # Convert OpenCV Images to ROS Image and publish it
        try:
            ros_image = self.bridge.cv2_to_imgmsg(
                depth, '32FC1', image_msg.header)
            self.depth_image_pub.publish(ros_image)
        except CvBridgeError as e:
            self.get_logger().error(
                f'Could not convert depth image to ROS: {e}')
            return

        # Republish the matching camera info, timestamped with the image header.
        last_camera_info = self.last_camera_info
        if last_camera_info is not None:
            camera_info_msg = copy.copy(last_camera_info)
            camera_info_msg.header = image_msg.header
            self.depth_camera_info_pub.publish(camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingROS()

    # A dedicated thread for the inference callback group keeps the node
    # responsive to lifecycle transitions while a heavy inference runs.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
