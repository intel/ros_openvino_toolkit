#!/usr/bin/env python
from vino_pipeline_srv_msgs.srv import *
import rospy
import sys
from pipeTree import TreeNode

def getMaping(pipeline):
    map_dict = dict()
    for pipeline in pipeline.connections:
        if pipeline.input not in map_dict.keys():
            map_dict[pipeline.input] = list()
        map_dict[pipeline.input].append(pipeline.output)

    return map_dict

def getTreeByMap(parent,input,map):
    if input not in map.keys():
        return parent
    for output in map[input]:
        child = parent.add_child(output)
        getTreeByMap(child,output,map)
    return parent


def getTree(parent,input,pipeline):
    map = getMaping(pipeline)
    return getTreeByMap(parent,input,map)

def reqPipelineService(cmd, value):
    rospy.wait_for_service('/openvino_toolkit/pipeline/service')
    try:
         
        req = rospy.ServiceProxy('/openvino_toolkit/pipeline/service', PipelineSrv)
        req_msg = PipelineSrvRequest()
        req_msg.pipeline_request.cmd = cmd
        req_msg.pipeline_request.value = value
        res = req(req_msg)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s <cmd> <value>"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        cmd = str(sys.argv[1])
        value = str(sys.argv[2])
    else:
        print usage()
        sys.exit(1)

    print("Requesting cmd:%s with value:%s"%(cmd, value))
    response = reqPipelineService(cmd, value)
    #print(response)
    for pipeline in response.pipelines:
        root = getTree(TreeNode(pipeline.name),'',pipeline)
        root.dump()

