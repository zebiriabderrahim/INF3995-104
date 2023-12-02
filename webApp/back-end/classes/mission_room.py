class MissionRoom:
    def __init__(self, host_id, robot=None, guest_id=None, type=None):
        self.host_id = host_id
        if type == 'simulation': # one simulated robot
            self.robot_info = Robot(robot["name"], robot["ipAddress"], "(simulation)")
        elif type == 'physical': # both physical robots
            self.robot_info = [Robot(bot["name"], bot["ipAddress"], "", bot["batteryLevel"]) for bot in robot]
        elif robot is None: # both simulated robots
            self.robot_info = Robot("Robots", "simulation", "(simulation)")
        else : # one physical robot
            self.robot_info = Robot(robot["name"], robot["ipAddress"], "", robot["batteryLevel"])
        self.guest_id = []
        
    def add_guest(self, guest_id):
        self.guest_id.append(guest_id)

    def to_dict(self):
        if isinstance(self.robot_info, list):
            return {
                "hostId": self.host_id,
                "robot": self.robot_info[0].to_dict(),
                "guestId": self.guest_id,
                "otherRobots": [robot.to_dict() for robot in self.robot_info[1:]] if len(self.robot_info) > 1 else []
            }
        else:
            return {
                "hostId": self.host_id,
                "robot": self.robot_info.to_dict(),
                "guestId": self.guest_id
            }


class Robot:
    def __init__(self, name, ip_address, simulation="", battery_level=None):
        self.name = name
        self.ip_address = ip_address
        self.state = "Active on mission" + simulation
        self.battery_level = battery_level

    def change_robot_state(self, state):
        self.state = state

    def to_dict(self):
        return {
            "name": self.name,
            "ipAddress": self.ip_address,
            "state": self.state,
            "batteryLevel": self.battery_level
        }
