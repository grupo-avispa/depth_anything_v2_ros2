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

"""Depth estimation domain logic, kept independent of rclpy for testability."""

from __future__ import annotations

import os

from depth_anything_v2.metric_depth.depth_anything_v2.dpt import DepthAnythingV2
import numpy as np
import torch

# Architecture parameters per encoder, as required by DepthAnythingV2.
ENCODER_CONFIGS = {
    'vits': {'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'features': 256, 'out_channels': [256, 512, 1024, 1024]},
}


def select_encoder_config(encoder: str) -> dict:
    """
    Return the DepthAnythingV2 architecture kwargs for the given encoder.

    Parameters
    ----------
    encoder : str
        Encoder name. Must be one of ``vits``, ``vitb`` or ``vitl``.

    Returns
    -------
    dict
        Keyword arguments (``features``, ``out_channels``) for ``DepthAnythingV2``.

    Raises
    ------
    ValueError
        If ``encoder`` is not one of the supported values.

    """
    config = ENCODER_CONFIGS.get(encoder)
    if config is None:
        allowed = ', '.join(sorted(ENCODER_CONFIGS))
        raise ValueError(f'Wrong encoder: [{encoder}]. Must be one of: {allowed}')
    return config


def resolve_device(requested: str, cuda_available: bool) -> str:
    """
    Resolve the torch device to use, falling back to CPU when CUDA is unavailable.

    Parameters
    ----------
    requested : str
        Device requested by configuration (e.g. ``cpu``, ``cuda:0``).
    cuda_available : bool
        Whether CUDA is available in the current environment.

    Returns
    -------
    str
        ``requested`` if it is ``cpu`` or CUDA is available, otherwise ``cpu``.

    """
    if requested != 'cpu' and not cuda_available:
        return 'cpu'
    return requested


def postprocess_depth(raw_depth: np.ndarray, max_depth: float) -> np.ndarray:
    """
    Clip a raw depth prediction to ``[0, max_depth]`` and cast it to float32.

    Parameters
    ----------
    raw_depth : np.ndarray
        Depth map as predicted by the model.
    max_depth : float
        Maximum depth, in meters, the checkpoint was trained with.

    Returns
    -------
    np.ndarray
        The clipped depth map, as ``float32``.

    """
    return np.clip(raw_depth, 0, max_depth).astype(np.float32)


class DepthEstimator:
    """
    Loads a metric DepthAnythingV2 model and runs inference on BGR images.

    This class has no ROS dependency so it can be unit tested and reused
    outside of a ROS node.

    Parameters
    ----------
    encoder : str
        Encoder to use for the model (``vits``, ``vitb`` or ``vitl``).
    model_file : str
        Path to the ``.pth`` checkpoint to load.
    device : str
        Torch device to run inference on (e.g. ``cpu``, ``cuda:0``).
    max_depth : float
        Maximum depth, in meters, the checkpoint was trained with.

    """

    def __init__(self, encoder: str, model_file: str, device: str, max_depth: float):
        self.encoder = encoder
        self.model_file = model_file
        self.device = device
        self.max_depth = max_depth
        self.model = None

    def load(self) -> None:
        """
        Build the model architecture and load its weights.

        Raises
        ------
        ValueError
            If ``self.encoder`` is not supported.
        FileNotFoundError
            If ``self.model_file`` does not exist.

        """
        config = select_encoder_config(self.encoder)
        self.model = DepthAnythingV2(encoder=self.encoder, max_depth=self.max_depth, **config)

        if not self.model_file or not os.path.isfile(self.model_file):
            raise FileNotFoundError(f'Model file not found: [{self.model_file}]')

        # weights_only=True avoids arbitrary code execution via pickle and is
        # required for forward compatibility with PyTorch >= 2.6.
        state_dict = torch.load(self.model_file, map_location=self.device, weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model.to(self.device).eval()

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        """
        Run metric depth inference on a BGR image.

        Parameters
        ----------
        bgr_image : np.ndarray
            Input image in BGR format, as expected by ``infer_image``.

        Returns
        -------
        np.ndarray
            The depth map, clipped to ``[0, max_depth]`` and cast to float32.

        """
        raw_depth = self.model.infer_image(bgr_image)
        return postprocess_depth(raw_depth, self.max_depth)
