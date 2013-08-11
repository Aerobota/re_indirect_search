#!/usr/bin/env python
 
import roslib; roslib.load_manifest('json_prolog')
 
import rospy
import json_prolog
 
if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    #query = prolog.query("owl_has(Obj, knowrob:describedInMap, 'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#SemanticEnvironmentMap0'), current_object_pose(Obj, Pose)")
    query = prolog.query("owl_has(Obj, knowrob:describedInMap, '/home/gajan/legacy_ws/re_indirect_search/maps/test-map.owl')")
    #query = prolog.query("owl_parse('/home/gajan/legacy_ws/re_indirect_search/maps/ias_semantic_map.owl', false, false, true), current_object_pose(Obj, Pose)")
    #query = prolog.query("owl_has(Obj, knowrob:describedInMap, '/home/gajan/legacy_ws/re_indirect_search/maps/ias_semantic_map.owl#SemanticEnvironmentMap0'), current_object_pose(Obj, Pose)")
    for solution in query.solutions():
        print solution
    query.finish()

