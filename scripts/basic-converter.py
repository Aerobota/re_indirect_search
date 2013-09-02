#!/usr/bin/env python
#
# You have to run [rosrun mod_semantic_map SemanticMapToOWL] to start the converter service
#

import roslib; roslib.load_manifest('re_indirect_search')
import rospy

from mod_semantic_map.msg import SemMap, SemMapObject
from mod_semantic_map.srv import GenerateSemanticMapOWL

def sem_map_to_OWL(sem_map):
	rospy.wait_for_service('/knowrob_semantic_map_to_owl/generate_owl_map')
	
	try:
		converter = rospy.ServiceProxy('/knowrob_semantic_map_to_owl/generate_owl_map', GenerateSemanticMapOWL)
		response = converter(sem_map)
		return response.owlmap
	except rospy.ServiceException as e:
		print('Service call failed: {0}'.format(e))


if __name__ == '__main__':
	rospy.init_node('basic_converter')

	print('basic-converter starting...')
	
	# create a semantic map
	sem_map = SemMap()
	sem_map.header.frame_id = "http://www.example.com/foo.owl#"
	sem_map.header.stamp = rospy.get_rostime()

	# create object - cupboard
	obj = SemMapObject()
	obj.id = 1
	obj.partOf = 0
	obj.type = 'cupboard'

	obj.depth = 1
	obj.width = 1
	obj.height = 1

	obj.pose = [1.0, 0.0, 0.0, 0.0,
			    0.0, 1.0, 0.0, 0.0,
			    0.0, 0.0, 1.0, 0.0,
			    0.0, 0.0, 0.0, 1.0]

	sem_map.objects.append(obj)

	# convert
	owl_string = sem_map_to_OWL(sem_map)
	print owl_string
	print('basic converter done')
