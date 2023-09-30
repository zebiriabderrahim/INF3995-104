class MissionRoom:
    def __init__(self, host_id, robot, guest_id=None):
        self.host_id = host_id
        self.robot_info = Robot(robot["name"], robot["ipAddress"], robot["batteryLevel"])
        self.guest_id = []
        
    def add_robot(self, robot_ip):
        self.robot_ips.append(robot_ip)
        
    def add_guest(self, guest_id):
        self.guest_id.append(guest_id)

    def to_dict(self):
        return {
            "hostId": self.host_id,
            "robot": self.robot_info.to_dict(),
            "guestId": self.guest_id
        }


class Robot:
    def __init__(self, name, ip_address, battery_level):
        self.name = name
        self.ip_address = ip_address
        self.state = "Active on mission"
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