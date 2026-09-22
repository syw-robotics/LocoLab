# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from collections.abc import Sequence

import torch
from isaaclab.utils.buffers import DelayBuffer

from .async_circular_buffer import AsyncCircularBuffer


class AsyncDelayBuffer(DelayBuffer):
    """Asynchronous delay buffer that allows retrieving stored data with delays asynchronously for each batch index."""

    def __init__(self, history_length: int, batch_size: int, device: str):
        """Initialize the asynchronous delay buffer.

        Args:
            history_length: The history of the buffer, i.e., the number of time steps in the past that the data
                will be buffered. It is recommended to set this value equal to the maximum time-step lag that
                is expected. The minimum acceptable value is zero, which means only the latest data is stored.
            batch_size: The batch dimension of the data.
            device: The device used for processing.
        """
        super().__init__(history_length, batch_size, device)
        self._circular_buffer = AsyncCircularBuffer(self._history_length + 1, batch_size, device)

    def compute(self, data: torch.Tensor, batch_ids: Sequence[int] | None = None) -> torch.Tensor:
        if batch_ids is None:
            return super().compute(data)
        else:
            if len(batch_ids) != data.shape[0]:
                raise ValueError(f"Batch IDs length {len(batch_ids)} does not match data shape {data.shape[0]}.")

        # add the new data to the last layer
        self._circular_buffer.append(data, batch_ids)
        # return the output
        delayed_data = self._circular_buffer.__getitem__(self._time_lags[batch_ids], batch_ids)
        return delayed_data.clone()
