import rospy
from control_planner import ControlPlanner
from mission_planner import MissionPlanner

class Planner():
  def __init__():
    self.planning_data = {
                  'mission': 'general', \
                  'test': 0 \
                  }

    self.mission_planner = MissionPlanner(self) #센서들과 통신. 미션 결정하고 무슨 데이터 받을 지 등등.
    print("====Mission Planner Start!")
    self.control_planner = ControlPlanner(self) #제어에서 뭐할지 결정 및 시리얼로 연결해서 직접 제어. 민수형 아이디어 채용.
    print("====Control Planner Start!")



  def run(self):
    rate=rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
            self.mission_planner.run()
            self.control_planner.run()
            rate.sleep()



if __name__ == "__main__":
    plan = Planner()
    plan.run()