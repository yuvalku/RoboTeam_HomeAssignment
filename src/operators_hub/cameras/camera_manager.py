
class CameraManager:
    def __init__(self,robot):
        self.robot_name = robot
        self.camera_lists = {
            "ROOK_1171": {
#--------------------ROOK 1171-------------------------------#
               # "rtsp://192.168.126.200:554/av0_": "MAST",
                "rtsp://192.168.125.136:554/av0_": "FRONT",
                "rtsp://192.168.125.137:554/av0_": "REAR",
                "rtsp://192.168.125.138:554/av0_": "RIGHT",
                "rtsp://192.168.125.139:554/av0_": "LEFT",
                "rtsp://192.168.125.140:554/av0_": "FRONT_RIGHT",
                "rtsp://192.168.125.141:554/av0_": "FRONT_LEFT",
            },
#------------------------------------------------------------#
            "TIGR_907": {
                "rtsp://192.168.29.89:554/av0_0": "GRIPPER",
                "rtsp://192.168.29.187:554/av0_0v": "FRONT",
                "rtsp://192.168.29.227:554/av0_0": "REAR",
            }
        }
        if robot.upper() not in self.camera_lists:
            raise ValueError(f"Unknown robot name: {robot}")

        self.cameras = self.camera_lists[robot]
        self.current_camera = None

    def list_cameras(self):
        for ip, description in self.cameras.items():
            print(f"{description}: {ip}")    

    def select_camera_by_description(self, description):
        for ip, desc in self.cameras.items():
            if desc.lower() == description.lower():
                self.current_camera = ip
                print(f"Camera Selected: {description}: {ip}")
                return ip
        print(f"Camera '{description}' not found")
        return None
    
    def toggle_camera(self):
        camera_ips = list(self.cameras.keys())  # Get a list of camera IPs
        if self.current_camera is None:
            self.current_camera = camera_ips[0]
        else:
            current_index = camera_ips.index(self.current_camera)
            next_index = (current_index + 1) % len(camera_ips)
            self.current_camera = camera_ips[next_index]

        return self.current_camera

    def get_current_camera(self):
        if self.current_camera:
            print(f"Current Camera: {self.current_camera}")
            return self.current_camera
        else:
            print("No camera selected")
            return None
        
    def get_initial_camera_ip(self):
        return next(iter(self.cameras.keys()))
