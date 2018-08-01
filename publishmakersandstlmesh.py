# -*- coding: utf-8 -*-
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from visualization_msgs.msg import Marker
import moveit_commander
import geometry_msgs
import rospy
import sys
from math import cos, sin, pi
from geometry_msgs.msg import Point
try:
    from pyassimp import pyassimp
    use_pyassimp = True
except:
    # In 16.04, pyassimp is busted
    # https://bugs.launchpad.net/ubuntu/+source/assimp/+bug/1589949
    use_pyassimp = False


def __make_sphere( name, pose, radius):
         co = CollisionObject()
         co.operation = CollisionObject.ADD
         co.id = name
         co.header = pose.header
         sphere = SolidPrimitive()
         sphere.type = SolidPrimitive.SPHERE
         sphere.dimensions = [radius]
         co.primitives = [sphere]
         co.primitive_poses = [pose.pose]
         return co

################################################################################################################################################
#这个程序要和ur5程序一起运行
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('offlineplanning',
                anonymous=True)

Ground_id = 'duck'
REFERENCE_FRAME = '/world'


scene = moveit_commander.PlanningSceneInterface() 
_psi = _moveit_planning_scene_interface.PlanningSceneInterface()
_pub_co = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=100) 
marker_pub = rospy.Publisher('visualization_marker', Marker, None, queue_size = 10)

pose_Ground = geometry_msgs.msg.PoseStamped()
pose_Ground.header.frame_id = REFERENCE_FRAME
pose_Ground.pose.position.x = 0.0
pose_Ground.pose.position.y = 0.0
pose_Ground.pose.position.z =  0.5 

scene.add_mesh(Ground_id,pose_Ground,'/home/yuan/doc/visual-pushing-grasping/DUCK.stl') 
scene_pub.publish()
######################################################################################3
#_pub_co.publish( __make_sphere('qiu', pose_Ground , 0.1))









points = Marker()
line_strip = Marker()
line_list = Marker()

points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/world"
points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time()
points.ns = line_strip.ns = line_list.ns = "points_and_lines"
points.action = line_strip.action = line_list.action = Marker.ADD
points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0

points.id = 0
line_strip.id = 1
line_list.id = 2

points.type = Marker.POINTS
line_strip.type = Marker.LINE_STRIP
line_list.type = Marker.LINE_LIST

# POINTS markers use x and y scale for width/height respectively
points.scale.x = 0.2
points.scale.y = 0.2

# LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
line_strip.scale.x = 0.1
line_list.scale.x = 0.1

# Points are green
points.color.g = 1.0
points.color.a = 1.0

# Line strip is blue
line_strip.color.b = 1.0
line_strip.color.a = 1.0

# Line list is red
line_list.color.r = 1.0
line_list.color.a = 1.0

for i in range(0,100):
	p = Point()
	p.x = i - 50
	points.points.append(p)
	line_strip.points.append(p)
	line_list.points.append(p)
	p2 = Point()
	p2.x = i - 50
	line_list.points.append(p2)

r = rospy.Rate(30)
f = 0

while not rospy.is_shutdown():

	for i in range(0,100):
	    # points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()

	    y = 5 * sin(f + i/100. * 2 * pi)
	    z = 5 * cos(f + i/100. * 2 * pi)

	    points.points[i].y = y
	    points.points[i].z = z

	    line_strip.points[i].y = y
	    line_strip.points[i].z = z

	    line_list.points[(i*2)].y = y
	    line_list.points[(i*2)].z = z

	    line_list.points[(i*2) + 1].y = y
	    line_list.points[(i*2) + 1].z = z + 1.0
	    
	marker_pub.publish(points)
	marker_pub.publish(line_strip)
	marker_pub.publish(line_list)

	r.sleep()

	f += 0.04;
