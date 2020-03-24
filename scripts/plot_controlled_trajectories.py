import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np
import datetime

SMALL_SIZE = 8
MEDIUM_SIZE = 12
BIGGER_SIZE = 22

plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=MEDIUM_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

#rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
### for Palatino and other serif fonts use:
##rc('font',**{'family':'serif','serif':['Palatino']})
#rc('text', usetex=True)

ref_file = '/home/swapneel/rosbags/csv/pd_2020-03-22-14-38-15-controller-reference.csv'
refn_file = '/home/swapneel/rosbags/csv/nmpc_2020-03-22-14-56-15-controller-reference.csv'
odom_pd_file = '/home/swapneel/rosbags/csv/pd_2020-03-22-14-38-15-ground_truth-state.csv'
odom_nmpc_file = '/home/swapneel/rosbags/csv/nmpc_2020-03-22-14-56-15-ground_truth-state.csv'
cmd_pd_file = '/home/swapneel/rosbags/csv/pd_2020-03-22-14-38-15-cmd_vel.csv'
cmd_nmpc_file = '/home/swapneel/rosbags/csv/nmpc_2020-03-22-14-56-15-cmd_vel.csv'

ref_df = pd.read_csv(ref_file)
refn_df = pd.read_csv(refn_file)
odom_pd_df = pd.read_csv(odom_pd_file)
odom_nmpc_df = pd.read_csv(odom_nmpc_file)
cmd_pd_df = pd.read_csv(cmd_pd_file)
cmd_nmpc_df = pd.read_csv(cmd_nmpc_file)

# Time indexing
#ref_df.time = pd.to_datetime(ref_df.time, format="%Y/%m/%d/%H:%M:%S.%f")
#refn_df.time = pd.to_datetime(refn_df.time, format="%Y/%m/%d/%H:%M:%S.%f")
#odom_pd_df.time = pd.to_datetime(odom_pd_df.time, format="%Y/%m/%d/%H:%M:%S.%f")
#odom_nmpc_df.time = pd.to_datetime(odom_nmpc_df.time, format="%Y/%m/%d/%H:%M:%S.%f")

#ref_df.time = pd.to_datetime(ref_df.time, format="%Y/%m/%d/%H:%M:%S.%f")
#refn_df.time = pd.to_datetime(refn_df.time, format="%Y/%m/%d/%H:%M:%S.%f")
#odom_pd_df.time = pd.to_datetime(odom_pd_df.time, format="%Y/%m/%d/%H:%M:%S.%f")
#odom_nmpc_df.time = pd.to_datetime(odom_nmpc_df.time, format="%Y/%m/%d/%H:%M:%S.%f")

ref_df['time_sec'] = ref_df['.stamp.secs'] + ref_df['.stamp.nsecs'] * 1e-9
refn_df['time_sec'] = refn_df['.stamp.secs'] + refn_df['.stamp.nsecs'] * 1e-9
odom_pd_df['time_sec'] = odom_pd_df['.header.stamp.secs'] + odom_pd_df['.header.stamp.nsecs'] * 1e-9
odom_nmpc_df['time_sec'] = odom_nmpc_df['.header.stamp.secs'] + odom_nmpc_df['.header.stamp.nsecs'] * 1e-9

ref_df = ref_df.set_index('time_sec')
refn_df = refn_df.set_index('time_sec')
odom_pd_df = odom_pd_df.set_index('time_sec')
odom_nmpc_df = odom_nmpc_df.set_index('time_sec')



odom_pd_df = odom_pd_df[ref_df.index[0]:] 
odom_nmpc_df = odom_nmpc_df[refn_df.index[0]:] 

time_ref = ref_df.index - ref_df.index[0]
#time_ref = np.array(pd.to_timedelta(time_ref).total_seconds())

time_refn = refn_df.index - refn_df.index[0]
#time_refn = np.array(pd.to_timedelta(time_refn).total_seconds())

time_odom_pd = odom_pd_df.index - odom_pd_df.index[0]
#time_odom_pd = np.array(pd.to_timedelta(time_odom_pd).total_seconds())

time_odom_nmpc = odom_nmpc_df.index - odom_nmpc_df.index[0]
#time_odom_nmpc = np.array(pd.to_timedelta(time_odom_nmpc).total_seconds())


#merged_df = ref_df.merge(odom_df, how='inner', on='time')

#['.position.x', '.position.y', '.position.z', '.velocity.x', '.velocity.y', '.velocity.z', '.yaw']
#['.header.seq', '.header.stamp.secs', '.header.stamp.nsecs', '.header.frame_id', '.child_frame_id', '.pose.pose.position.x', '.pose.pose.position.y', '.pose.pose.position.z', '.pose.pose.orientation.x', '.pose.pose.orientation.y', '.pose.pose.orientation.z', '.pose.pose.orientation.w', '.pose.covariance', '.twist.twist.linear.x', '.twist.twist.linear.y', '.twist.twist.linear.z', '.twist.twist.angular.x', '.twist.twist.angular.y', '.twist.twist.angular.z', '.twist.covariance']


xref = np.array(ref_df['.position.x'])
yref = np.array(ref_df['.position.y'])
zref = np.array(ref_df['.position.z'])
vxref = np.array(ref_df['.velocity.x'])
vyref = np.array(ref_df['.velocity.y'])
vzref = np.array(ref_df['.velocity.z'])

xrefn = np.array(refn_df['.position.x'])
yrefn = np.array(refn_df['.position.y'])
zrefn = np.array(refn_df['.position.z'])
vxrefn = np.array(refn_df['.velocity.x'])
vyrefn = np.array(refn_df['.velocity.y'])
vzrefn = np.array(refn_df['.velocity.z'])

x_pd = np.array(odom_pd_df['.pose.pose.position.x'])
y_pd = np.array(odom_pd_df['.pose.pose.position.y'])
z_pd = np.array(odom_pd_df['.pose.pose.position.z'])
vx_pd = np.array(odom_pd_df['.twist.twist.linear.x'])
vy_pd = np.array(odom_pd_df['.twist.twist.linear.y'])
vz_pd = np.array(odom_pd_df['.twist.twist.linear.z'])

x_nmpc = np.array(odom_nmpc_df['.pose.pose.position.x'])
y_nmpc = np.array(odom_nmpc_df['.pose.pose.position.y'])
z_nmpc = np.array(odom_nmpc_df['.pose.pose.position.z'])
vx_nmpc = np.array(odom_nmpc_df['.twist.twist.linear.x'])
vy_nmpc = np.array(odom_nmpc_df['.twist.twist.linear.y'])
vz_nmpc = np.array(odom_nmpc_df['.twist.twist.linear.z'])


x_pd_error = xrefn - x_pd[:xrefn.shape[0]]
y_pd_error = yrefn - y_pd[:yrefn.shape[0]]
z_pd_error = zrefn - z_pd[:zrefn.shape[0]]
x_nmpc_error = xrefn - x_nmpc[:xrefn.shape[0]]
y_nmpc_error = yrefn - y_nmpc[:yrefn.shape[0]]
z_nmpc_error = zrefn - z_nmpc[:zrefn.shape[0]]

vx_pd_error = vxrefn - vx_pd[:vxrefn.shape[0]]
vy_pd_error = vyrefn - vy_pd[:vyrefn.shape[0]]
vz_pd_error = vzrefn - vz_pd[:vzrefn.shape[0]]
vx_nmpc_error = vxrefn - vx_nmpc[:vxrefn.shape[0]]
vy_nmpc_error = vyrefn - vy_nmpc[:vyrefn.shape[0]]
vz_nmpc_error = vzrefn - vz_nmpc[:vzrefn.shape[0]]


# Plot trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs=xref, ys=yref, zs=zref, color='blue', linestyle='dashed')
ax.plot(xs=x_pd, ys=y_pd, zs=z_pd, color='red')
ax.plot(xs=x_nmpc, ys=y_nmpc, zs=z_nmpc, color='green')
ax.legend(['Reference', 'PD Control','NMPC'], loc='upper center')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_xlim(-5.5, 5.5)
ax.set_ylim(-5.5, 5.5)
ax.set_zlim(0, 2)

# Plot x, y, z vs t
fig_pos = plt.figure()
ax_pos = fig_pos.add_subplot(311)
ax_pos.plot(time_refn, xrefn, 'b--')
ax_pos.plot(time_odom_pd, x_pd, 'r-')
ax_pos.plot(time_odom_nmpc, x_nmpc, 'g-')
ax_pos.set_xlabel('t (s)')
ax_pos.set_ylabel('X (m)')

ax_pos = fig_pos.add_subplot(312)
ax_pos.plot(time_refn, yrefn, 'b--')
ax_pos.plot(time_odom_pd, y_pd, 'r-')
ax_pos.plot(time_odom_nmpc, y_nmpc, 'g-')
ax_pos.set_xlabel('t (s)')
ax_pos.set_ylabel('Y (m)')

ax_pos = fig_pos.add_subplot(313)
ax_pos.plot(time_refn, zrefn, 'b--')
ax_pos.plot(time_odom_pd, z_pd, 'r-')
ax_pos.plot(time_odom_nmpc, z_nmpc, 'g-')
ax_pos.set_xlabel('t (s)')
ax_pos.set_ylabel('Z (m)')

fig_pos.legend(['Reference', 'PD Control','NMPC'], loc='upper right')
fig_pos.suptitle('Position (m) vs time (s)')

# Plot vx, vy, vz vs t
fig_vel = plt.figure()
ax_vel = fig_vel.add_subplot(311)
ax_vel.plot(time_refn,vxrefn, 'b--')
ax_vel.plot(time_odom_pd,vx_pd, 'r-')
ax_vel.plot(time_odom_nmpc,vx_nmpc, 'g-')
#ax_vel.set_xlabel('t (s)')
ax_vel.set_ylabel('Vx (m/s)')

ax_vel = fig_vel.add_subplot(312)
ax_vel.plot(time_refn,vyrefn, 'b--')
ax_vel.plot(time_odom_pd,vy_pd, 'r-')
ax_vel.plot(time_odom_nmpc,vy_nmpc, 'g-')
#ax_vel.set_xlabel('t (s)')
ax_vel.set_ylabel('Vy (m/s)')

ax_vel = fig_vel.add_subplot(313)
ax_vel.plot(time_refn,vzrefn, 'b--')
ax_vel.plot(time_odom_pd,vz_pd, 'r-')
ax_vel.plot(time_odom_nmpc,vz_nmpc, 'g-')
ax_vel.set_xlabel('t (s)')
ax_vel.set_ylabel('Vz (m/s)')

fig_vel.legend(['Reference', 'PD Control','NMPC'], loc='upper right')
fig_vel.suptitle('Velocity (m/s) vs time (s)')


# Error histograms
fig_hist_x = plt.figure()
ax_hist_x = fig_hist_x.add_subplot(131)
ax_hist_x.hist(x_pd_error, bins=100, color='red', alpha=0.8)
ax_hist_x.hist(x_nmpc_error, bins=100, color='green', alpha=0.8)
ax_hist_x.set_xlabel('X error (m)')
ax_hist_x.set_ylabel('Frequency')

ax_hist_y = fig_hist_x.add_subplot(132)
ax_hist_y.hist(y_pd_error, bins=100, color='red', alpha=0.8)
ax_hist_y.hist(y_nmpc_error, bins=100, color='green', alpha=0.8)
ax_hist_y.set_xlabel('Y error (m)')
#ax_hist_y.set_ylabel('Frequency')

ax_hist_z = fig_hist_x.add_subplot(133)
ax_hist_z.hist(z_pd_error, bins=100, color='red', alpha=0.8)
ax_hist_z.hist(z_nmpc_error, bins=100, color='green', alpha=0.8)
ax_hist_z.set_xlabel('Z error (m)')
#ax_hist_z.set_ylabel('Frequency')
fig_hist_x.legend(['PD Control','NMPC'], loc='upper right')
fig_hist_x.suptitle('Position Tracking Error Histogram')


fig_hist_v = plt.figure()
ax_hist_x = fig_hist_v.add_subplot(131)
ax_hist_x.hist(vx_pd_error, bins=100, color='red', alpha=0.8)
ax_hist_x.hist(vx_nmpc_error, bins=100, color='green', alpha=0.8)
ax_hist_x.set_xlabel('$V_x$ error (m/s)')
ax_hist_x.set_ylabel('Frequency')

ax_hist_y = fig_hist_v.add_subplot(132)
ax_hist_y.hist(vy_pd_error, bins=100, color='red', alpha=0.8)
ax_hist_y.hist(vy_nmpc_error, bins=100, color='green', alpha=0.8)
ax_hist_y.set_xlabel('$V_y$ error (m/s)')
#ax_hist_y.set_ylabel('Frequency')

ax_hist_z = fig_hist_v.add_subplot(133)
ax_hist_z.hist(vz_pd_error, bins=100, color='red', alpha=0.8)
ax_hist_z.hist(vz_nmpc_error, bins=100, color='green', alpha=0.8)
ax_hist_z.set_xlabel('$V_z$ error (m/s)')
#ax_hist_z.set_ylabel('Frequency')
fig_hist_v.legend(['PD Control','NMPC'], loc='upper right')
fig_hist_v.suptitle('Velocity Tracking Error Histogram')



print(cmd_nmpc_df.shape[0])
cmd_time = np.linspace(0, 60.0, cmd_nmpc_df.shape[0])

fig_cmd = plt.figure()
ax_theta = fig_cmd.add_subplot(311)
ax_theta.plot(cmd_time, cmd_nmpc_df['.linear.y'], color='blue')
ax_theta.set_ylabel(r'$ {\theta}_{des}$ (rad)')
ax_phi = fig_cmd.add_subplot(312)
ax_phi.plot(cmd_time, cmd_nmpc_df['.linear.x'], color='red')
ax_phi.set_ylabel(r'$\phi_{des}$ (rad)')
ax_thrust = fig_cmd.add_subplot(313)
ax_thrust.plot(cmd_time, cmd_nmpc_df['.linear.z'], color='green')
ax_thrust.set_ylabel(r'${\tau}_{des}$ $(m/s^{2})$')
ax_thrust.set_xlabel('t (s)')
fig_cmd.suptitle('NMPC command inputs')

plt.show()
