import paramiko
from services import ros_utilities as ros

filepaths = {}

def get_robot_files(password):
    """
    Retrieve files from the remote server based on a provided password.

    This function checks the provided password and, if valid, proceeds to retrieve the contents of
    various robot-related files from the remote server. It utilizes helper functions for file retrieval.

    :param password: The password to authenticate the request.
    :return: A dictionary containing filenames as keys and file contents as values, or 'unauthorized' if the password is invalid.
    """
    if password == 'nvidia':
        filepaths = read_robot_filenames()
        file_contents = {}
        
        for filepath in filepaths:
            
            content = fetch_file_from_remote(filepaths[filepath])
            if content is not None:
                filename = filepath.split('/')[-1]
                file_contents[filename] = content
        return file_contents
    else:
        return 'unauthorized'


def save_robot_files(password, files):
    """
    Saves files on remote servers associated with specific IP addresses after authentication.

    This function validates the provided password and, if authorized, saves the given files' content
    on two different IP addresses. It uses predefined file paths to identify the locations for saving
    the files. Once the files are saved, it triggers a ROS node on both IP addresses to handle the saved files.

    :param password: The password for authorization.
    :param files: A dictionary containing filenames as keys and their respective contents as values.
    :return: 'success' if the files are saved successfully, 'unauthorized' otherwise.
    """
    if password == 'nvidia':
        for filename, content in files.items():
            filepath = filepaths[filename]
            save_file_on_remote('192.168.0.110', filepath, content)
            save_file_on_remote('192.168.0.122', filepath, content)
        ros.execute_command({'ipAddress':'192.168.0.110'}, 'roslaunch /home/nvidia/INF3995-104/embedded/agilex_ws/src/save_test_file_pkg/launch/save_test.launch') 
        ros.execute_command({'ipAddress':'192.168.0.122'}, 'roslaunch /home/nvidia/INF3995-104/embedded/agilex_ws/src/save_test_file_pkg/launch/save_test.launch') 
        return 'success'
    else:
        return 'unauthorized'


def read_robot_filenames():
    """
    Reads and retrieves robot file paths from a file.

    This function reads the 'robotfilepaths.txt' file, containing full paths for robot files.
    It parses each line, extracting the filename and full path, storing them in a global variable 'filepaths'.

    If 'filepaths' is empty, it populates it by extracting filenames and paths from the file.
    Once the content is extracted or if 'filepaths' is already populated, it returns the stored filepaths.

    :return: A dictionary containing filenames as keys and their corresponding full paths as values.
    """
    global filepaths
    if len(filepaths) == 0:
        with open('robotfilepaths.txt', 'r') as file:
            for line in file:
                fullpath = line.strip()
                filename = fullpath.split('/')[-1]
                filepaths[filename] = fullpath
    return filepaths


def fetch_file_from_remote(filepath):
    """
    Fetches the content of a file from a remote server using SSH.

    This function connects to a remote server with specified credentials via SSH and retrieves
    the content of the file located at the given 'filepath'. It uses the paramiko library for SSH operations.

    :param filepath: The full path of the file on the remote server.
    :return: The content of the file as a string, or None if an error occurs during the SSH operation.
    """
    try:
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        ssh_client.connect(hostname='192.168.0.110', port=22, username='nvidia', password='nvidia')

        stdin, stdout, stderr = ssh_client.exec_command(f'cat {filepath}')
        file_content = stdout.read().decode('utf-8')

        ssh_client.close()

        return file_content
    except Exception as e:
        print(f"Error reading thru SSH : {e}")
        return None


def save_file_on_remote(ip, filepath, content):
    """
    Saves content to a file on a remote machine using SSH.

    This function establishes an SSH connection to a remote machine specified by 'ip' using the 'paramiko' library.
    It writes the provided 'content' to the specified 'filepath' on the remote machine using the established SSH connection.

    :param ip: The IP address of the remote machine.
    :param filepath: The path to the file on the remote machine where the content will be written.
    :param content: The content to be written to the remote file.
    :return: None if an error occurs during the SSH operation, otherwise, no explicit return.
    """
    try:
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        ssh_client.connect(hostname=ip, port=22, username='nvidia', password='nvidia')

        ssh_client.exec_command(f'echo "{content}" > {filepath}')

        ssh_client.close()
    except Exception as e:
        print(f"Error writing thru SSH : {e}")
        return None