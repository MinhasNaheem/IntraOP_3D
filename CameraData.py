import requests


class CameraData():
    def __init__(self,toolName,refName):
        if isinstance(toolName, str) and isinstance(refName, str):
            self.geometry = [toolName, refName]
        else:
            self.geometry = []
    
    def GetCameraData(self):
        data={}
        try:
            url2 = 'http://127.0.0.1:8081/GetCameraData'
            r = requests.get(url2,timeout=0.1)
            data = r.json()
        except ConnectionError:
            print("Camera not connected")
        except requests.exceptions.RequestException as e:  
            print("Camera not connected")

        return data

 
    def parseCameraData(self,camData):
        
        RegisteredMarkerCount = 0
        data = {}
        try:
            json_dict = camData
            RegisteredMarkerCount =  len(
                    json_dict['RegisteredMarkersList'])
        except:
            print('json error')
            print(f'printing self.cam data {json_dict}')
        if  RegisteredMarkerCount != 0 and bool(self.geometry): 
            for i in range(RegisteredMarkerCount):
                for Markers in self.geometry: 
                    if (json_dict['RegisteredMarkersList']
                        [i]["MarkerName"]) == Markers:
                        Marker0 = {}
                        Marker0 = json_dict['RegisteredMarkersList'][i]
                        rot = Marker0['Top']['rotation']
                        pos = Marker0['Top']['point']
                        err_fre = Marker0['ErrorValue']
                        position = [pos['x'],pos['y'],pos['z'] ] 
                        quat = [ rot['x'],rot['y'],rot['z'],rot['w']]

                        data[Markers] = (quat,position,err_fre) 
                        

        else:
            print("Marker not visible")
            
        
        return data