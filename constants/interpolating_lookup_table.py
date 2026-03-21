from bisect import bisect_left


class InterpolatingLookupTable:
    """Linear interpolation over distance-indexed lookup points."""

    def __init__(self) -> None:
        # Keep keys and values in parallel sorted lists so bisect can find
        # insertion and interpolation bounds in O(log n) time.
        self._keys: list[float] = []
        self._values: list[float] = []

    def __len__(self) -> int:
        return len(self._keys)

    def put(self, key: float, value: float) -> None:
        index = bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            # Overwrite existing calibration points instead of duplicating them.
            self._values[index] = value
            return

        self._keys.insert(index, key)
        self._values.insert(index, value)

    def get(self, key: float) -> float | None:
        if not self._keys:
            return None

        index = bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            return self._values[index]
        # Clamp to the nearest endpoint outside the calibrated range.
        if index == 0:
            return self._values[0]
        if index >= len(self._keys):
            return self._values[-1]

        low_key = self._keys[index - 1]
        high_key = self._keys[index]
        low_value = self._values[index - 1]
        high_value = self._values[index]

        if high_key == low_key:
            return low_value

        # Blend linearly between the surrounding calibration points.
        blend = (key - low_key) / (high_key - low_key)
        return low_value + (high_value - low_value) * blend

    def clear(self) -> None:
        self._keys.clear()
        self._values.clear()
