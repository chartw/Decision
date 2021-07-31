from geometry_msgs.msg import Point32
from math import hypot
import time
from lib.planner_utils.mapping import Mapping
from master_node.msg import Path



class MissionPlan:
    def __init__(self, planner):

        self.obstacle_msg = planner.obstacle_msg
        self.object_msg = planner.object_msg
        self.local = planner.local
        self.surface_msg = planner.surface_msg
        self.serial_msg = planner.serial_msg
        self.parking_msg = planner.parking_msg
        self.object_msg = planner.object_msg
        self.mission_ing = planner.mission_ing  # True / False
        self.base = []
        self.base.append(Point32(22.760400877965, 41.7303388307402, 0))
        self.base.append(Point32(17.978170358155626, 34.84945192598553, 0))
        self.mode=""

        self.parking_lot = []
        self.parking_lot.append(Point32(17.623907356361915, 41.175622253568505, 0))
        self.parking_lot.append(Point32(15.85266480396189, 38.844924089730185, 0))
        self.parking_lot.append(Point32(14.16998329088652, 36.736197374027405, 0))
        self.parking_lot.append(Point32(12.398738836681156, 34.405499957494584, 0))
        self.parking_lot.append(Point32(10.716055698028432, 32.18578849457638, 0))

        self.time_count = 0
        self.temp_heading = 0

        self.start_time = time.time()
        self.current_time = time.time()

    def decision(self, planner):
        mission=planner.global_path.mission[planner.veh_index]
        env=planner.global_path.env[planner.veh_index]
        dist=planner.planning_msg.dist

        print(mission, env, dist)
        if mission=="small" or mission=="big":
            if dist >0:
                self.mode = mission
            elif len(planner.local_path.x)!=0:
                if hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y) < 3:
                    planner.local_path=Path()
                    self.mode="general"
            else:
                self.mode="general"

        elif mission=="kid":
            if env=="bump":
                self.mode="bump"
            elif dist > 0 and dist < 5:
                self.mode="emergency_stop"
            else:
                self.mode="kid"

        else:
            self.mode="general"

        return self.mode

