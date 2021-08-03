import sys
class StopLine:
    nst_stop=0

    def stop_idx_check(self,planner):
        veh=planner.veh_index
        global_path=planner.global_path

        if self.nst_stop > veh:
            return self.nst_stop

        else:
            for index in range(veh,len(global_path.x)-1):
                if global_path.env[index]=="stop":
                    self.nst_stop=index
                    return self.nst_stop

        return int(sys.maxsize)

    