"""Periodic centerline, asymmetric map corridors and independent footprint checks."""
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import distance_transform_edt
import yaml
from PIL import Image


class Track:
    def __init__(self, rows, config):
        self.config = config
        rows = np.asarray(rows, dtype=float)
        if rows.ndim != 2 or rows.shape[1] != 4 or len(rows) < 8 or not np.isfinite(rows).all():
            raise ValueError('centerline requires >=8 finite x,y,right_width,left_width rows')
        if np.linalg.norm(rows[0, :2] - rows[-1, :2]) < 1e-6:
            rows = rows[:-1]
        length = np.linalg.norm(np.roll(rows[:, :2], -1, axis=0) - rows[:, :2], axis=1)
        if np.any(length < 1e-5) or np.any(length > 1.) or np.any(rows[:, 2:] <= 0):
            raise ValueError('discontinuous centerline or invalid widths')
        s = np.r_[0., np.cumsum(length)]
        spline = CubicSpline(s, np.vstack([rows[:, :2], rows[0, :2]]), bc_type='periodic')
        # Reparameterize by measured spline arc length; retain conservative width
        # minima of original endpoints, never cubic-interpolate boundary widths.
        dense = np.linspace(0., s[-1], max(100, int(s[-1]/0.02)+1))
        xy = spline(dense)
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        if np.any(np.diff(arc) < 1e-8):
            raise ValueError('degenerate centerline spline')
        self.length = float(arc[-1])
        self.spline = CubicSpline(arc, xy, bc_type='periodic')
        self.s = arc[:-1]
        self.xy = xy[:-1]
        ix = np.minimum(np.searchsorted(s, dense[:-1], side='right')-1, len(rows)-1)
        self.width = np.minimum(rows[ix, 2:], rows[(ix+1) % len(rows), 2:])
        if np.any(self.width <= config.body_width/2 + config.clearance):
            raise ValueError('track too narrow for configured footprint and clearance')
        _, _, _, kappa, _, _ = self.sample(self.s)
        cap = np.minimum(config.nominal_speed, np.sqrt(config.lateral_acceleration /
                                                  np.maximum(abs(kappa), 1e-5)))
        cap = np.minimum(cap*config.speed_scale, config.max_speed)
        # Cyclic backward braking envelope previews corners beyond the MPC horizon.
        # Two loops suffice: information can propagate once around the whole track.
        ds = np.diff(arc)
        for _ in range(2):
            for i in range(len(cap)-1, -1, -1):
                cap[i] = min(cap[i], math.sqrt(cap[(i+1) % len(cap)]**2 +
                                              2*config.deceleration*0.7*ds[i]))
        self.speed = cap
        self.tangent = self.sample(self.s)[1]
        self.curvature = kappa

    @classmethod
    def load(cls, path, config):
        return cls(np.loadtxt(path, delimiter=',', ndmin=2), config)

    def sample(self, theta):
        s = np.asarray(theta) % self.length
        xy = self.spline(s)
        derivative = self.spline(s, 1)
        norm = np.linalg.norm(derivative, axis=-1)
        if not np.isfinite(norm).all() or np.any(norm < 1e-6):
            raise ValueError('singular track tangent')
        tangent = derivative / np.maximum(norm[..., None], 1e-9)
        normal = np.stack([-tangent[..., 1], tangent[..., 0]], axis=-1)
        dd = self.spline(s, 2)
        curvature = (derivative[..., 0]*dd[..., 1] - derivative[..., 1]*dd[..., 0]) / norm**3
        ix = np.clip(np.searchsorted(self.s, s, side='right')-1, 0, len(self.s)-1)
        return xy, tangent, normal, curvature, self.width[ix, 0], self.width[ix, 1]

    def speed_limit(self, theta):
        return np.interp(np.asarray(theta) % self.length,
                         np.r_[self.s, self.length], np.r_[self.speed, self.speed[0]])

    def project(self, point, yaw=None, previous=None, window=None):
        """Nearest local projection, with heading filtering and unwrapped progress."""
        point = np.asarray(point)
        distance = np.linalg.norm(self.xy-point, axis=1)
        candidates = np.ones(len(distance), dtype=bool)
        if previous is not None:
            arc = (self.s-previous+self.length/2) % self.length-self.length/2
            candidates &= abs(arc) <= (self.config.search_window if window is None else window)
        if yaw is not None:
            candidates &= self.tangent @ np.array([math.cos(yaw), math.sin(yaw)]) > 0
        if not candidates.any():
            raise ValueError('no aligned local track projection')
        i = np.argmin(np.where(candidates, distance, np.inf))
        theta = float(self.s[i])
        if previous is not None:
            theta = previous + (theta-previous+self.length/2) % self.length-self.length/2
        # Short Newton projection, bounded to the chosen local segment neighborhood.
        anchor = theta
        for _ in range(3):
            r, t, _, _, _, _ = self.sample(theta)
            theta = float(np.clip(theta + (point-r) @ t, anchor-.05, anchor+.05))
        r, t, n, _, _, _ = self.sample(theta)
        return theta, float((point-r) @ n), float((point-r) @ t)

    def parameters(self, theta):
        r, t, n, _, right, left = self.sample(theta)
        return np.r_[r, t, n, theta, right, left, self.speed_limit(theta)]


class FreeMap:
    """Map-server free mask; unknown and outside cells are unsafe."""
    def __init__(self, yaml_path):
        with open(yaml_path) as stream:
            meta = yaml.safe_load(stream)
        self.resolution = float(meta['resolution'])
        self.origin = np.asarray(meta['origin'], dtype=float)
        if not .005 <= self.resolution <= .5 or not np.isfinite(self.origin).all():
            raise ValueError('invalid map metadata')
        path = Path(yaml_path).parent / meta['image']
        pixels = np.asarray(Image.open(path).convert('L'))
        probability = pixels/255. if meta.get('negate', 0) else (255.-pixels)/255.
        self.free = probability < float(meta.get('free_thresh', .25))
        if not meta.get('negate', 0) and meta.get('mode', 'trinary') == 'trinary':
            self.free &= pixels != 205
        # Padding makes map edges occupied for distance-transform clearance.
        padded = np.pad(self.free, 1, constant_values=False)
        self.distance = distance_transform_edt(padded)[1:-1, 1:-1]*self.resolution

    def clearance(self, points):
        points = np.asarray(points)-self.origin[:2]
        c, s = np.cos(self.origin[2]), np.sin(self.origin[2])
        u = (c*points[..., 0]+s*points[..., 1])/self.resolution
        v = (-s*points[..., 0]+c*points[..., 1])/self.resolution
        col = np.floor(u).astype(int)
        row = self.free.shape[0]-1-np.floor(v).astype(int)
        inside = (row >= 0) & (row < self.free.shape[0]) & (col >= 0) & (col < self.free.shape[1])
        result = np.full(row.shape, -self.resolution)
        # Conservative whole-cell bound, independent of subpixel location.
        result[inside] = self.distance[row[inside], col[inside]]-math.sqrt(2)*self.resolution
        return result


def footprint_points(states, config, spacing):
    """Dense body samples, including all edges/corners and the interior."""
    x = np.linspace(-config.body_rear, config.body_front,
                    max(2, int(np.ceil((config.body_front+config.body_rear)/spacing))+1))
    y = np.linspace(-config.body_width/2, config.body_width/2,
                    max(2, int(np.ceil(config.body_width/spacing))+1))
    xx, yy = np.meshgrid(x, y)
    body = np.column_stack([xx.ravel(), yy.ravel()])
    states = np.atleast_2d(states)
    c, s = np.cos(states[:, 2]), np.sin(states[:, 2])
    result = np.empty((len(states), len(body), 2))
    result[:, :, 0] = states[:, None, 0]+c[:, None]*body[:, 0]-s[:, None]*body[:, 1]
    result[:, :, 1] = states[:, None, 1]+s[:, None]*body[:, 0]+c[:, None]*body[:, 1]
    return result
