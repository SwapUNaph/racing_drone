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


file_prefix = "~/rosbags/csv/estimation_2020-03-24-12-40-57"
#file_prefix = "~/rosbags/csv/estimation_2020-03-24-10-44-10"
#file_prefix = "~/rosbags/csv/estimation_2020-03-23-19-50-53"


# extract covariances
def extract_covariances(df):
	if ('.pose.covariance' in df.columns):
		df['.pose.covariance'] = df['.pose.covariance'].apply(lambda x: map(float, x[1:-1].split(',')))
		df['covX'] = df['.pose.covariance'].apply(lambda x: x[0])
		df['covY'] = df['.pose.covariance'].apply(lambda x: x[4])
		df['covZ'] = df['.pose.covariance'].apply(lambda x: x[8])
		#print(df.covX)
		return df
	else:
		return df
	
	
# Load and clean data to plot
def load_and_clean(file_prefix):
	file_suffixes = ["-gate_detector-filtered_gate_pose.csv", "-gate_detector-raw_gate_pose.csv",
						"-localizer-actual_gate_pose.csv", "-ground_truth-state.csv", "-localizer-estimated_state.csv"]
	
	data_frames = []
	out_frames = []
	for suff in file_suffixes:
		df = pd.read_csv(file_prefix + suff)
		data_frames.append(df)
		#print(df.columns)
		print("File loaded: %s\n" % file_prefix+suff)
	
	#print(df['.header.stamp.secs'][0])
	start_time = 163.0
	
	for df in data_frames:
		# u'.header.stamp.secs', u'.header.stamp.nsecs'
		#print(df.columns)
		df['time_sec'] = df['.header.stamp.secs'] + df['.header.stamp.nsecs'] * 1e-9 - start_time
		df = df.set_index('time_sec')
		df = df[0:]
		df = df.reset_index()
		#df = df[ ~( (df['time_sec'] > 5.0) & (df['time_sec'] < 5.5) ) & ~( (df['time_sec'] > 15.6) & (df['time_sec'] < 15.9) ) & ~( (df['time_sec'] > 24.56) & (df['time_sec'] < 24.89) ) & ~( (df['time_sec'] > 34.59) & (df['time_sec'] < 35.24) )] 
		#print(df.time_sec)
		df = extract_covariances(df)
		out_frames.append(df)
		del df
		
	return out_frames
 
# Get gate coordinates for plotting
def get_gate_coordinates(gate):
	hs = 0.75
	gate_coord = np.array( [[ gate[0], gate[0], gate[0], gate[0], gate[0], gate[0], gate[0] ], 
	                       [gate[1]+hs, gate[1]-hs, gate[1]-hs, gate[1]+hs, gate[1]+hs, gate[1], gate[1] ],
	                        [gate[2]-hs, gate[2]-hs, gate[2]+hs, gate[2]+hs, gate[2]-hs, gate[2]-hs, gate[2]-2.25]])
	return gate_coord
	


# Get estimation error
def get_est_error(gt, est):
	#print(gt.shape, est.shape)
	#gt.shape > est.shape
	gt2 = gt.set_index('time_sec')
	est2 = est.set_index('time_sec')
	t = np.array(est2.index)
	error = []
	for i in range(len(t)):
		if i+1 < len(t):
			error.append( [ gt2['.pose.pose.position.x'][t[i]-0.1:t[i+1]+0.1].mean() - est2['.pose.pose.position.x'][t[i]], gt2['.pose.pose.position.y'][t[i]-0.1:t[i+1]+0.1].mean() - est2['.pose.pose.position.y'][t[i]], gt2['.pose.pose.position.z'][t[i]-0.1:t[i+1]+0.1].mean() - est2['.pose.pose.position.z'][t[i]] ] )
	
	error = filter(lambda x: type(x[0]) == type(np.float64(20)), error)
	del gt2, est2
	return np.array(error)



frames = load_and_clean(file_prefix)



estimation_error = get_est_error(frames[3],frames[4])
#estimation_error = estimation_error[~np.isnan(estimation_error).any(axis=1)]
#print(estimation_error)


# Plot trajectory and gates
gate1 = get_gate_coordinates([4,-1,2.25])
gate2 = get_gate_coordinates([-4,1,2.25])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs=frames[3][".pose.pose.position.x"], ys=frames[3][".pose.pose.position.y"], zs=frames[3][".pose.pose.position.z"], color='blue', linestyle='dashed')
ax.plot(xs=frames[4][".pose.pose.position.x"], ys=frames[4][".pose.pose.position.y"], zs=frames[4][".pose.pose.position.z"], color='red')
ax.plot(xs=gate1[0,:], ys=gate1[1,:], zs=gate1[2,:], color='#FF9900', linewidth=3)
ax.plot(xs=gate2[0,:], ys=gate2[1,:], zs=gate2[2,:], color='#FF9900', linewidth=3)
ax.legend(['Ground Truth', 'Estimated Trajectory', 'Gates'], loc='upper center')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_zlim(0, 4)

# Plot gate position
fig_pos = plt.figure()
ax_pos = fig_pos.add_subplot(311)
ax_pos.plot(frames[2].time_sec, frames[2]['.pose.position.x'], 'b--', linewidth=2)
ax_pos.plot(frames[1].time_sec, frames[1]['.pose.position.x'], 'rx')
ax_pos.plot(frames[0].time_sec, frames[0]['.pose.pose.position.x'], 'g-', linewidth=2)
ax_pos.set_ylabel('X (m)')

ax_pos = fig_pos.add_subplot(312)
ax_pos.plot(frames[2].time_sec, frames[2]['.pose.position.y'], 'b--', linewidth=2)
ax_pos.plot(frames[1].time_sec, frames[1]['.pose.position.y'], 'rx')
ax_pos.plot(frames[0].time_sec, frames[0]['.pose.pose.position.y'], 'g-', linewidth=2)
ax_pos.set_ylabel('Y (m)')

ax_pos = fig_pos.add_subplot(313)
ax_pos.plot(frames[2].time_sec, frames[2]['.pose.position.z'], 'b--', linewidth=2)
ax_pos.plot(frames[1].time_sec, frames[1]['.pose.position.z'], 'rx')
ax_pos.plot(frames[0].time_sec, frames[0]['.pose.pose.position.z'], 'g-', linewidth=2)
ax_pos.set_xlabel('t (s)')
ax_pos.set_ylabel('Z (m)')

fig_pos.legend(['Ground Truth', 'Measured','Estimated'], loc='upper right')
fig_pos.suptitle('Gate Position wrt Drone (m) vs time (s)')



#Plot gate position covariance
fig_cov = plt.figure()
ax_pos = fig_cov.add_subplot(311)
ax_pos.plot(frames[0].time_sec, frames[0].covX, 'g-', linewidth=2) 
ax_pos.set_ylabel('Var X ($m^{2}$)')

ax_pos = fig_cov.add_subplot(312)
ax_pos.plot(frames[0].time_sec, frames[0].covY, 'g-', linewidth=2)
ax_pos.set_ylabel('Var Y ($m^{2}$)')

ax_pos = fig_cov.add_subplot(313)
ax_pos.plot(frames[0].time_sec, frames[0].covZ, 'g-', linewidth=2)
ax_pos.set_ylabel('Var Z ($m^{2}$)')
ax_pos.set_xlabel('t (s)')

fig_cov.suptitle('Gate Position Estimation Variance ($m^{2}$) vs time (s)')


##Plot drone position covariance
#fig_cov = plt.figure()
#ax_pos = fig_cov.add_subplot(311)
#ax_pos.plot(frames[4].time_sec, frames[0].covX) 
#ax_pos.set_ylabel('X (m)')

#ax_pos = fig_cov.add_subplot(312)
#ax_pos.plot(frames[4].time_sec, frames[0].covY)
#ax_pos.set_ylabel('Y (m)')

#ax_pos = fig_cov.add_subplot(313)
#ax_pos.plot(frames[4].time_sec, frames[0].covZ)
#ax_pos.set_ylabel('Z (m)')

#fig_cov.suptitle('Drone Position Estimation Variance (m) vs time (s)')

# Plot drone position
fig_pos_d = plt.figure()
ax_pos = fig_pos_d.add_subplot(311)
ax_pos.plot(frames[3].time_sec, frames[3]['.pose.pose.position.x'], 'b--', linewidth=2)
ax_pos.plot(frames[4].time_sec, frames[4]['.pose.pose.position.x'], 'r-', linewidth=2)
ax_pos.set_ylabel('X (m)')

ax_pos = fig_pos_d.add_subplot(312)
ax_pos.plot(frames[3].time_sec, frames[3]['.pose.pose.position.y'], 'b--', linewidth=2)
ax_pos.plot(frames[4].time_sec, frames[4]['.pose.pose.position.y'], 'r-', linewidth=2)
ax_pos.set_ylabel('Y (m)')

ax_pos = fig_pos_d.add_subplot(313)
ax_pos.plot(frames[3].time_sec, frames[3]['.pose.pose.position.z'], 'b--', linewidth=2)
ax_pos.plot(frames[4].time_sec, frames[4]['.pose.pose.position.z'], 'r-', linewidth=2)
ax_pos.set_ylim(0,4)
ax_pos.set_ylabel('Z (m)')
ax_pos.set_xlabel('t (s)')

fig_pos_d.legend(['Ground Truth', 'Estimated'], loc='upper right')
fig_pos_d.suptitle('Drone Position (m) vs time (s)')

#Error plots
fig_hist_x = plt.figure()

ax_hist_x = fig_hist_x.add_subplot(131)
ax_hist_x.hist(estimation_error[:,0], bins=100, color='green', alpha=0.8)
ax_hist_x.set_xlabel('X error (m)')
ax_hist_x.set_ylabel('Frequency')

ax_hist_x = fig_hist_x.add_subplot(132)
ax_hist_x.hist(estimation_error[:,1], bins=100, color='green', alpha=0.8)
ax_hist_x.set_xlabel('Y error (m)')

ax_hist_x = fig_hist_x.add_subplot(133)
ax_hist_x.hist(estimation_error[:,2], bins=100, color='green', alpha=0.8)
ax_hist_x.set_xlabel('Z error (m)')

fig_hist_x.suptitle('Estimation Error Histogram')

plt.show()

