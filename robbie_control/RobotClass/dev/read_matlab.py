import scipy.io

lookup_table = scipy.io.loadmat('100_height_waypoints')
print lookup_table['waypoints']