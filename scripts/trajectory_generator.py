import numpy as np

# Make trajectory file
filename = "trajectory.yaml"

# x,y,z,vx,vy,vz,yaw,state_typ, state_change_typ, state_change_threshold
# state_typ:
#   GATE : 0
#   WAYPOINT: 1
# state_change_type
#   TIME : 0
#   DISTANCE : 1

T = 30.0 # s
step = 300.0
dt = T/step

initial_wait = 10.0
thresh = dt
state_type = 1
state_change_type = 0
f = 3/T
A = 5.0



t = np.linspace(0,T,step,endpoint=True)

X = A * np.cos(2*np.pi*f * t)
Y = A * np.sin(2*np.pi*f * t)
Z = np.ones(t.shape[0])
VX = -A * 2*np.pi*f * np.sin(2*np.pi*f * t)
VY = A * 2*np.pi*f * np.cos(2*np.pi*f * t)
VZ =  0.0 * t
yaw = 0.0 * t

print("x:", X)
print("y:", Y)
print("z:", Z)
print("VX:", VX)
print("VY:", VY)
print("VZ:", VZ)
print("yaw: ", yaw)


    

with open(filename, 'w') as fHandle:
  fHandle.write("trajectory:\n  states: [\n    %.2f,    0.00,    1.00,    0.00,    0.00,    0.00,    0.00,    1.00,    0.00,    %.2f,\n" % (A, initial_wait))
  for i, k in enumerate(t):
    x = X[i]
    y = Y[i]
    z = Z[i]
    vx = VX[i]
    vy = VY[i]
    vz = VZ[i]
    yw = yaw[i]

    if i == len(t)-1:
      vx = 0.0
      vy = 0.0
      vz = 0.0
      state_type = 0
      thresh = 1000.0

    fHandle.write("    %.2f,    %.2f,    %.2f,    %.2f,    %.2f,    %.2f,    %.2f,    %.2f,     %.2f,    %.2f,\n" % (x, y, z, vx, vy, vz, yw, state_type, state_change_type, thresh ))
  fHandle.write("  ]\n")
