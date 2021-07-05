import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import pcl.pcl_visualization
'''
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth)

pipeline.start(config)

pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2)

# spatial filter
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_smooth_alpha, 0.4)
spatial.set_option(rs.option.filter_smooth_delta, 20)
# temporal filter
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.5)
# threshold filter

threshold = rs.threshold_filter(0.2, 0.8)

frame = pipeline.wait_for_frames()
depth_frame = frame.get_depth_frame()
depth_frame = decimate.process(depth_frame)
depth_frame = threshold.process(depth_frame)
depth_frame = spatial.process(depth_frame)
depth_frame = temporal.process(depth_frame)


points = pc.calculate(depth_frame)
vertex = np.asarray(points.get_vertices(2))

'''
# ---------------------------------------------------
# normalEstimator = pcl.NormalEstimation(pointXYZ)
# normalEstimator.compute()
# viewer = pcl.pcl_visualization.PCLVisualizering()

point = []
with open("data.txt") as data:
    for line in data.readlines():
        # print(line)
        line = line.replace('\n', '')
        line = line.split(' ')
        if float(line[1]) < 0.2:
            continue
        point.append([float(line[0]), float(line[1])])

path = np.array(point)
length = len(path)
print(f"Selected {length} points.")
y = path[:,0]
originalZ = path[:,1]

# moving average
windowSize = 20
filteredZ = np.convolve(originalZ, np.ones((windowSize,))/windowSize, mode='same')

# select smooth zone
trajectory = []
for i in range(len(y)):
    if y[i] < 0.15 and y[i] > -0.02:
        trajectory.append([y[i], filteredZ[i]])
trajectory = np.array(trajectory)

#  environment information for impedance control
# TODO: 数据噪声太大, 得想一个好的微分的办法. 可以将间隔点取成均匀的, 滤波效果会好很多.
ze_dot = []
for i in range(len(trajectory)-1):
    ze_d = (trajectory[i+1,1] - trajectory[i, 1]) / (trajectory[i+1,0] - trajectory[i,0])
    if abs(ze_d) > 10:
        continue 
    ze_dot.append([trajectory[i,0], ze_d])
ze_dot = np.array(ze_dot)
ze_dot_filtered = np.convolve(ze_dot[:,1], np.ones((20,))/20, mode="same")

# np.arange()

# robot simulation
# Plot
plt.figure()
plt.xlabel("y/m")
plt.ylabel("z/m")
plt.title("Section")
plt.plot(y, originalZ, 'r')
plt.plot(y, filteredZ, 'b')
plt.plot(trajectory[:,0], trajectory[:,1], 'g-')
plt.legend(["Original data", "Moving averaged data", "Trajectory"])

plt.figure()
plt.xlabel("y/m")
plt.ylabel("$\dot z$")
plt.title("1st order derivative")
plt.plot(ze_dot[:,0], ze_dot[:,1], 'r')
plt.plot(ze_dot[:,0], ze_dot_filtered, 'g')
plt.legend(["$\dot z_e$", "filtered_$\dot z_e$"])

plt.show()