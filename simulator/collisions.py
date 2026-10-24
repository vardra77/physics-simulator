import numpy as np
import numpy.typing as npt
import trimesh
import scipy as sci
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import sympy as sp

collision_manager = trimesh.collision.CollisionManager()