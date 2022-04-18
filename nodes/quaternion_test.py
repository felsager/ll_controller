import numpy as np
import quaternion

q = quaternion.from_euler_angles(np.pi, 0, 0)
print(q)
print(q.z)