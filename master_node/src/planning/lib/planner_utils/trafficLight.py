from numpy import argmax, array

class trafficLight:
    def __init__(self):
        self.lights_count = {'Green':0, 'Red': 0, 'Yellow':0, 'RedLeft':0, 'GreenLeft':0}
        self.light_names = ['Green', 'Red', 'Yellow', 'RedLeft', 'GreenLeft']
        self.ratio_th = 1.5
        self.size_th = 2
        self.queue = []
        self.queue_size=10

        self.result_mapping={'0':'A1', '1':'A2', '2':'A3', '3':'B1', '4':'B2', '5':'B3', '6':' Red', '7':'Yellow', '8' : 'RedLeft', '9': 'GreenLeft', '10':'Green'}


    def run(self, msg):
        detections = msg.detections

        max_size = 0
        max_class = 'Red' #Default
        for detection in detections:
    
            sign_id = str(detection.results.id)
            sign_name = self.result_mapping[sign_id]


            # print(detection)
            if sign_name in list(self.lights_count.keys()):
                x = detection.bbox.size_x
                y = detection.bbox.size_y
                ratio = y/x
                if ratio > self.ratio_th:
                    continue # 버스 컷
                

                size = x*y
                if size > max_size:
                    max_size = size
                    max_class = sign_name
        
        self.queue.append(max_class)
        self.lights_count[max_class]+=1
        print(self.lights_count)
        if len(self.queue) >self.queue_size:
            # print('================')
            traffic_name=self.queue.pop(0)
            # print('pop', traffic_name)
            self.lights_count[traffic_name]-=1
            

        max_traffic = max(self.lights_count, key=self.lights_count.get)

        print(self.lights_count.values())
        print(max_traffic)
        # return self.light_names[max_traffic]
        return max_traffic