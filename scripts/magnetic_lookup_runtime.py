"""Runtime lookup contract shared by benchmark drivers and validation tools.

It intentionally raises ``LookupCoverageError`` on every out-of-range pose.
Callers must stop/rebuild the table; extrapolation is never implicit.
"""
from pathlib import Path
import numpy as np


class LookupCoverageError(RuntimeError):
    pass


class MagneticLookup:
    names = ("x", "y", "z", "roll", "pitch", "yaw", "phase")

    def __init__(self, path):
        path = Path(path)
        with np.load(path, allow_pickle=True) as f:
            self.data = np.asarray(f["data"], float)
            self.columns = tuple(str(x) for x in f["columns"])
            self.axes = {n: np.unique(self.data[:, i]) for i, n in enumerate(self.names)}
        self._rows = {tuple(np.round(row[:7], 13)): row[7:13] for row in self.data}
        if self.data.shape[1] != 13 or self.columns[-6:] != ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"):
            raise ValueError("Unsupported magnetic lookup schema")

    def _bracket(self, name, value):
        axis = self.axes[name]
        if value < axis[0] - 1e-14 or value > axis[-1] + 1e-14:
            raise LookupCoverageError(f"{name}={value:g} outside [{axis[0]:g}, {axis[-1]:g}]")
        j = int(np.searchsorted(axis, value, side="right"))
        return max(0, min(j - 1, len(axis) - 2)), min(j, len(axis) - 1)

    def evaluate(self, position_m, euler_rad, phase_deg):
        query = np.r_[position_m, euler_rad, phase_deg]
        brackets = [self._bracket(n, float(v)) for n, v in zip(self.names, query)]
        # Multilinear interpolation over the regular grid stored row-major.
        out = np.zeros(6)
        for mask in range(1 << len(self.names)):
            weight = 1.0; key = []
            for v, (lo, hi), n in zip(query, brackets, self.names):
                a = self.axes[n]; frac = 0.0 if hi == lo else (v - a[lo]) / (a[hi] - a[lo])
                if mask & (1 << len(key)):
                    key.append(a[hi]); weight *= frac
                else:
                    key.append(a[lo]); weight *= 1.0 - frac
            if abs(weight) < 1e-12: continue
            values = self._rows.get(tuple(np.round(key, 13)))
            if values is None:
                # CSV/NPZ decimal round-trips can differ by a few ulps.
                dist = np.max(np.abs(self.data[:, :7] - np.asarray(key)), axis=1)
                j = int(np.argmin(dist))
                if dist[j] > 1e-6: raise RuntimeError("Corrupt/non-unique magnetic lookup grid")
                values = self.data[j, 7:13]
            out += weight * values
        return out[:3], out[3:]
