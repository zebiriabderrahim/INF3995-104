from services import ros_utilities as ros


def draw_on_screen(robot_ip):
    return ros.send_command(robot_ip, 'draw_on_screen.launch')

def play_sound(robot_ip):
    return ros.send_command(robot_ip, 'play_sound.launch')

def identify_robot(robot_ip):
    if robot_ip == '198.162.0.122':
        return draw_on_screen(robot_ip)
    if robot_ip == '192.168.0.110':
        return play_sound(robot_ip)            
  
