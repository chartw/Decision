from numpy import argmax, array

class trafficLight:
    def __init__(self):
        self.lights_count = {'Green':0, 'Red': 0, 'Yellow':0, 'RedLeft':0, 'GreenLeft':0}
        self.lights_names = ['Green', 'Red', 'Yellow', 'RedLeft', 'GreenLeft']
        self.ratio_th = 1.5
        self.size_th = 2
        self.queue = []
        self.queue_size=6



    def run(self, boxes):
        max_size = 0
        max_box = None
        for box in boxes:
            if box.Class in list(self.lights_count.keys()):
                x = box.xmax - box.xmin
                y = box.ymax - box.ymin
                ratio = y/x
                if ratio > self.ratio_th:
                    continue # 버스 컷
                

                size = x*y
                if size > max_size:
                    max_size = size
                    max_box = box

        self.queue.append(max_box.Class)
        self.lights_count[max_box.Class]+=1
        if len(self.queue) >self.queue_size:
            temp=self.queue.pop(0)
            self.lights_count[temp.Class]-=1
            

        
        max_traffic = argmax(array(self.lights_count.values()))
        
        return self.light_name[max_traffic]