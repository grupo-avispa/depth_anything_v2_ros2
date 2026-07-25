=========
CHANGELOG
=========

All notable changes to the depth_anything_v2_ros2 project will be documented in this file.

[1.1.0] - 25-07-2026
=====================

Added
-----
- Added ``depth_anything_v2_ros2/depth_estimator.py`` with the pure depth estimation domain
  logic, extracted out of the ROS node so it can be unit tested without rclpy, a GPU or a
  checkpoint file:

  - ``select_encoder_config()`` maps an encoder name to its ``DepthAnythingV2`` architecture
    kwargs and raises ``ValueError`` on an unsupported value.
  - ``resolve_device()`` falls back to CPU when CUDA is requested but unavailable.
  - ``postprocess_depth()`` clips a raw depth prediction to ``[0, max_depth]`` and casts it
    to ``float32``.
  - ``DepthEstimator`` loads the model (raising ``FileNotFoundError``/``ValueError`` on a
    missing checkpoint or invalid encoder) and runs inference.

- Added ``test/test_depth_estimator.py`` with 8 pytest cases covering
  ``select_encoder_config()``, ``resolve_device()`` and ``postprocess_depth()``.
- Migrated ``DepthAnythingROS`` from ``rclpy.node.Node`` to ``rclpy.lifecycle.LifecycleNode``:
  the model is now loaded in ``on_configure`` (returning ``TransitionCallbackReturn.FAILURE``
  on an invalid encoder or missing checkpoint instead of crashing with an opaque
  ``AttributeError``), and the subscribers/publishers are created in ``on_activate`` and torn
  down in ``on_deactivate``/``on_cleanup``.
- Added an ``autostart`` launch argument (default ``true``) in ``launch/default.launch.py``,
  passed to the ``launch_ros.actions.LifecycleNode`` action so it configures and activates
  ``depth_anything`` right after startup.
- Added ``camera_info_topic``/``depth_camera_info_topic`` parameters: the node now
  republishes the input ``CameraInfo`` alongside the depth image (timestamped with the
  image header), so downstream nodes such as ``depth_image_proc`` can pair both topics to
  build a point cloud.
- Added a ``MutuallyExclusiveCallbackGroup`` for ``image_callback`` and switched ``main()``
  to a ``MultiThreadedExecutor``, so a heavy inference no longer blocks lifecycle
  transitions and other callbacks.
- Documented the ``depth_anything_v2`` submodule update procedure in ``README.md``.

Changed
-------
- ``package.xml``: added ``python3-numpy``, ``python3-opencv`` and ``nav2_common`` (already
  used by the launch file, previously undeclared) as ``exec_depend``.
- ``requirements.txt``: removed ``gradio``, ``gradio_imageslider`` and ``huggingface_hub``
  (unused, pulled in from the upstream Gradio demo).
- ``.github/workflows/build.yml``: removed ``skip-tests: true`` so ``ament_flake8``,
  ``ament_pep257``, ``ament_copyright`` and the new pytest suite actually run in CI.
- ``torch.load()`` now uses ``weights_only=True`` and validates that ``model_file`` exists
  before loading, instead of deserializing an unrestricted pickle and failing with a
  low-level traceback on a missing file.
- ``max_depth`` is now validated to be strictly positive in ``get_params()``.
- Extracted the repeated declare/get/log parameter pattern into a ``_declare()`` helper.
- Fixed the ``image_callback`` docstring (it described "detections", copied from an
  unrelated node) and the misleading "Invert depth valies" comment.
- Replaced ``print(e)`` with ``self.get_logger().error(...)`` in the ``CvBridgeError``
  handler for the outgoing publish, so failures show up in the ROS logs.
- Removed the duplicated ``start_time = time.time()`` call and the unnecessary
  ``self.current_image`` instance attribute (now a local variable).
- Removed trailing whitespace flagged by ``ament_flake8`` (W293).

[1.0.0] - previous releases
============================

See the git history prior to this file for changes up to the metric depth support.
