import rospy
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def rotate_point(cx, cy, angle, p):
    s = np.sin(angle)
    c = np.cos(angle)

    # translate point back to origin:
    p[0] -= cx
    p[1] -= cy

    # rotate point
    xnew = p[0] * c - p[1] * s
    ynew = p[0] * s + p[1] * c

    # translate point back
    p[0] = xnew + cx
    p[1] = ynew + cy
    return p


def rotate_points(points, cx, cy, angle):
    result_points = []
    for p in points:
        result_points.append(rotate_point(cx, cy, angle, p))
    return result_points


def translate(points, x, y):
    result_points = []
    for p in points:
        result_points.append([p[0] + x, p[1] + y])
    return result_points


def main():
    points = []
    rot = -np.pi/2
    lim = 25
    ox = oy = lim//2
    rotation = np.pi/32

    for i in range(lim):
        pt = [i, i]
        if i > ox:
            pt = rotate_point(ox, oy, rot, [i, i])

        pt.append(0)
        points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "map"
    pcl_scan = point_cloud2.create_cloud(header, fields, points)
    header = Header()
    header.frame_id = "map"
    rotated_points = rotate_points(points, 0, 0, rotation)
    pcl_model = point_cloud2.create_cloud(header, fields, rotated_points)

    while not rospy.is_shutdown():
        pcl_scan.header.stamp = rospy.Time.now()
        pub_scan.publish(pcl_scan)
        rospy.sleep(1)
        pcl_model.header.stamp = rospy.Time.now()
        pub_model.publish(pcl_model)
        rospy.loginfo("Published pcl: {} and {} deg".format(0, rotation))


if __name__ == '__main__':
    rospy.init_node("create_pcl2")
    pub_scan = rospy.Publisher("scan_cloud", PointCloud2, queue_size=2)
    pub_model = rospy.Publisher("model_cloud", PointCloud2, queue_size=2)
    rospy.loginfo("Starting node")
    main()