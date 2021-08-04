from numpy import argmax, array

class deliveryClass:
    def __init__(self):
        self.sign_count = {'A1':0, 'A2': 0, 'A3':0, 'B1':0, 'B2':0, 'B3':0}
        self.sign_names = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3']
        self.ratio_th = 1.5
        self.size_th = 2
        self.queue = []
        self.queue_size=10
        self.order_list = ['B1', 'B2', 'B3']
###
    def detect_signs(self, boxes):
        for box in boxes:
            if box.Class in list(self.sign_count.keys()): #A1~B3
                
                

    def sign_order(self, boxes):
        

        pass
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