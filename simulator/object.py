from dataclasses import dataclass, field, asdict
import numpy as np
import numpy.typing as npt
import trimesh
import scipy as sci
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import sympy as sp

import main
import collisions

@dataclass
class Object:
    name: str = field(init=False)
    position: npt.NDArray[np.float64]
    orientation: npt.NDArray[np.float64]
    velocity: npt.NDArray[np.float64]
    angular_velocity: npt.NDArray[np.float64]
    mass: float
    mesh: trimesh.Trimesh
    inertia: npt.NDArray[np.float64] = field(init=False)

    x_prev: npt.NDArray[np.float64] = field(init=False)
    q_prev: npt.NDArray[np.float64] = field(init=False)

    normal: npt.NDArray[np.float64] = field(init=False)
    tension: npt.NDArray[np.float64] = field(init=False)

    charge: float = 0.0
    movable: bool = True

    def __post_init__(self):
        self.name = f"Object_{id(self)}"
        main.objects.append(self)

        self.x_prev = self.position.copy()
        self.q_prev = self.orientation.copy()

        self.inertia = self.mesh.moment_inertia
        self.mesh.apply_translation(self.position-self.mesh.center_mass)
        self.mesh.apply_transform(R.from_quat(self.orientation).as_matrix())
        collisions.collision_manager.add_object(self.name, self.mesh)

    def update(self, dx, dq, dv, dw):
        self.x_prev = self.position.copy()
        self.q_prev = self.orientation.copy()
        self.position += dx
        self.orientation += dq
        self.orientation = self.orientation / np.linalg.norm(self.orientation)
        self.velocity += dv
        self.angular_velocity += dw

        transform = np.eye(4)
        transform[:3, :3] = R.from_quat(self.orientation - self.q_prev).as_matrix()
        transform[:3, 3] = self.position - self.x_prev
        collisions.collision_manager.set_transform(self.name, transform)

    def state(self):
        return asdict(self)

    def get_transform(self):
        transform = np.eye(4)
        transform[:3, :3] = R.from_quat(self.orientation).as_matrix()
        transform[:3, 3] = self.position
        return transform