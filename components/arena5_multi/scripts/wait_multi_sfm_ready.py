#!/usr/bin/env python3
"""Wait for ten acknowledged display updates before starting an acceptance probe."""
import argparse
import json
import time
import rclpy
from std_msgs.msg import String

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--timeout',type=float,default=300);args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('wait_multi_sfm_ready');state={}
    def status(message):
        try:state.update(json.loads(message.data))
        except ValueError:pass
    node.create_subscription(String,'/multirobot/hunav/status',status,10)
    deadline=time.monotonic()+args.timeout;code=1
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if state.get('fault'):break
        if state.get('healthy') and state.get('compute',0)>=10 and state.get('updates',0)>=10:
            code=0;break
    print(json.dumps({'ready':code==0,'status':state},sort_keys=True))
    node.destroy_node();rclpy.shutdown();return code
if __name__=='__main__':raise SystemExit(main())
