#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     scripts/cleaner.py
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2013 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#

import sys

import roslib; roslib.load_manifest('re_indirect_search')
import rospy

from re_srvs.srv import DelEnvironment, SearchEnvironments


def main(api_key):
    print('wait for "re_comm/search_environments"...')
    rospy.wait_for_service('re_comm/search_environments')
    search = rospy.ServiceProxy('re_comm/search_environments', SearchEnvironments)
    
    print('wait for "re_comm/del_environment"...')
    rospy.wait_for_service('re_comm/del_environment')
    delete = rospy.ServiceProxy('re_comm/del_environment', DelEnvironment)
    
    print('look for environment maps to delete')
    uids = search('NYU_Depth_Dataset_V2.Scene_ID-').uids
    
    print('{} environment maps to delete'.format(len(uids)))
    
    for uid in uids:
        if delete(uid, api_key).success:
            print('Removal of {} successful'.format(uid))
        else:
            print('Removal of {} failed'.format(uid))
    
    print('done')
    
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('cleaner.py [API key]')
        exit(0)
    
    main(sys.argv[1])
