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

"""Unit tests for the pure depth estimation domain logic.

These tests exercise `select_encoder_config`, `resolve_device` and
`postprocess_depth` without requiring a GPU, a checkpoint file or rclpy,
since none of them touch the model itself.
"""

import numpy as np
import pytest

from depth_anything_v2_ros2.depth_estimator import (
    ENCODER_CONFIGS,
    postprocess_depth,
    resolve_device,
    select_encoder_config,
)


@pytest.mark.parametrize('encoder', ['vits', 'vitb', 'vitl'])
def test_select_encoder_config_returns_known_configs(encoder):
    """Each supported encoder must resolve to its documented architecture."""
    assert select_encoder_config(encoder) == ENCODER_CONFIGS[encoder]


def test_select_encoder_config_rejects_unknown_encoder():
    """An unsupported encoder must raise ValueError instead of failing later."""
    with pytest.raises(ValueError, match='vitx'):
        select_encoder_config('vitx')


def test_select_encoder_config_rejects_empty_encoder():
    """The empty string (the params.yaml default) must also be rejected."""
    with pytest.raises(ValueError):
        select_encoder_config('')


def test_resolve_device_keeps_cpu():
    """CPU is always a valid device, regardless of CUDA availability."""
    assert resolve_device('cpu', cuda_available=False) == 'cpu'
    assert resolve_device('cpu', cuda_available=True) == 'cpu'


def test_resolve_device_keeps_cuda_when_available():
    """The requested CUDA device is kept when CUDA is available."""
    assert resolve_device('cuda:0', cuda_available=True) == 'cuda:0'


def test_resolve_device_falls_back_to_cpu_when_cuda_unavailable():
    """A CUDA request falls back to CPU when CUDA is not available."""
    assert resolve_device('cuda:0', cuda_available=False) == 'cpu'


def test_postprocess_depth_clips_to_max_depth():
    """Values above max_depth must be clipped, not just cast."""
    raw_depth = np.array([-1.0, 0.0, 5.0, 10.0, 20.0], dtype=np.float64)

    result = postprocess_depth(raw_depth, max_depth=10.0)

    np.testing.assert_array_equal(result, [0.0, 0.0, 5.0, 10.0, 10.0])


def test_postprocess_depth_returns_float32():
    """The published image is 32FC1, so the array must be float32."""
    raw_depth = np.zeros((2, 2), dtype=np.float64)

    result = postprocess_depth(raw_depth, max_depth=10.0)

    assert result.dtype == np.float32
