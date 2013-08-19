#!/usr/bin/env python
# 
# You have to run [rosrun mod_semantic_map SemanticMapToOWL] to start the converter service
# 


import roslib; roslib.load_manifest('re_indirect_search')

import rospy
from mod_semantic_map.msg import SemMap, SemMapObject
from mod_semantic_map.srv import GenerateSemanticMapOWL

def SemMapToOWL(semmap):
	rospy.wait_for_service('/knowrob_semantic_map_to_owl/generate_owl_map');
	try:
		converter = rospy.ServiceProxy('/knowrob_semantic_map_to_owl/generate_owl_map', GenerateSemanticMapOWL)
		response = converter(semmap)
		return response.owlmap
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


if __name__ == '__main__':
	rospy.init_node('basic_converter')

	print 'basic-converter starting...'
	## create a semantic map	
	semmap = SemMap()
	semmap.header.frame_id = "http://www.example.com/foo.owl#"
	semmap.header.stamp = rospy.get_rostime()
	
	#create object - cupboard
	obj1 = SemMapObject()
	obj1.id = 1
	obj1.partOf=0
	obj1.type='cupboard'

	obj1.depth=1
	obj1.width=1
	obj1.height=1

	obj1.pose = [1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]
	
	semmap.objects.append(obj1)

	#convert
	owl_string = SemMapToOWL(semmap)
	print owl_string

	print 'basic converter done'
