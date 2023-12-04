import unittest
from unittest.mock import patch, call, Mock
from flask import Flask
from controllers import routes
from services import robot_update


class RobotUpdate(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.app.register_blueprint(routes.main)
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()
        self.robot_update = robot_update
        self.robot_update.filepaths = {}

    def tearDown(self):
        self.robot_update.filepaths = {}

    @patch('services.robot_update.fetch_file_from_remote')
    @patch('services.robot_update.read_robot_filenames')
    def test_get_robot_files(self, mock_read, mock_fetch):
        mock_read.return_value = {'nvidia/File1.py': 'File content'}
        mock_fetch.return_value = 'File content'
        result = self.robot_update.get_robot_files('nvidia')

        self.assertEqual(result, {'File1.py': 'File content'})

    @patch('services.robot_update.fetch_file_from_remote')
    @patch('services.robot_update.read_robot_filenames')
    def test_get_robot_files_error(self, mock_read, mock_fetch):
        result = self.robot_update.get_robot_files('hoohaa')
        self.assertEqual(result, 'unauthorized')


    @patch('services.ros_utilities.execute_command')
    @patch('services.robot_update.save_file_on_remote')
    def test_save_robot_files(self, mock_save, mock_execute_command):
        self.robot_update.filepaths['File.py'] = 'nvidia/File.py'
        result = self.robot_update.save_robot_files('nvidia', {'File.py': 'content'})
        mock_save.assert_has_calls([
            call('192.168.0.110', 'nvidia/File.py', 'content'),
            call('192.168.0.122', 'nvidia/File.py', 'content')
        ])
        mock_execute_command.assert_has_calls([
            call({'ipAddress':'192.168.0.110'}, 'roslaunch /home/nvidia/INF3995-104/embedded/agilex_ws/src/save_test_file_pkg/launch/save_test.launch'),
            call({'ipAddress':'192.168.0.122'}, 'roslaunch /home/nvidia/INF3995-104/embedded/agilex_ws/src/save_test_file_pkg/launch/save_test.launch')
        ])
        self.assertEqual(result, 'success')


    def test_save_robot_files_error(self):
        result = self.robot_update.save_robot_files('hoohaa', {})
        self.assertEqual(result, 'unauthorized')


    def test_read_robot_filenames(self):
        result = self.robot_update.read_robot_filenames()
        self.assertEqual(result, {
            'save_test_file.py': '/home/nvidia/INF3995-104/embedded/agilex_ws/src/save_test_file_pkg/src/save_test_file.py',
            'start_script.sh': '/home/nvidia/INF3995-104/start_script.sh'
        })


    @patch('paramiko.AutoAddPolicy')
    @patch('paramiko.SSHClient')
    def test_fetch_file_from_remote(self, mock_ssh_client, mock_policy):
        mock_ssh = mock_ssh_client.return_value
        mock_policy.return_value = None
        mock_stdin = Mock()
        mock_stdout = Mock()
        mock_stderr = Mock()
        expected_output = 'Hello World!'
        mock_stdout.read.return_value.decode.return_value = expected_output
        mock_ssh.exec_command.return_value = mock_stdin, mock_stdout, mock_stderr

        result = self.robot_update.fetch_file_from_remote('nvidia/File.py')

        mock_policy.assert_called_once()
        mock_ssh.set_missing_host_key_policy.assert_called_once_with(None)
        mock_ssh.connect.assert_called_once_with(hostname='192.168.0.110', port=22, username='nvidia', password='nvidia')
        mock_ssh.exec_command.assert_called_once_with('cat nvidia/File.py')
        mock_ssh.close.assert_called_once()

        self.assertEqual(result, expected_output)


    @patch('paramiko.SSHClient')
    def test_fetch_file_from_remote_exception(self, mock_ssh_client):
        mock_ssh_client.side_effect = Exception('')
        with patch('builtins.print') as mock_print:
            result = self.robot_update.fetch_file_from_remote('nvidia/File.py')
            mock_print.assert_called_once_with("Error reading thru SSH : ")
            self.assertEqual(result, None)


    @patch('paramiko.SSHClient')
    @patch('paramiko.AutoAddPolicy')
    def test_save_file_on_remote(self, mock_policy, mock_ssh_client):
        mock_ssh = mock_ssh_client.return_value
        mock_policy.return_value = None

        self.robot_update.save_file_on_remote('192.168.0.110', 'nvidia/File.py', 'Sample content')

        mock_policy.assert_called_once()
        mock_ssh.set_missing_host_key_policy.assert_called_once_with(None)
        mock_ssh.connect.assert_called_once_with(hostname='192.168.0.110', port=22, username='nvidia', password='nvidia')
        mock_ssh.exec_command.assert_called_once_with(f'echo "Sample content" > nvidia/File.py')
        mock_ssh.close.assert_called_once()


    @patch('paramiko.SSHClient')
    def test_save_file_on_remote_exception(self, mock_ssh_client):
        mock_ssh_client.side_effect = Exception('')
        with patch('builtins.print') as mock_print:
            result = self.robot_update.save_file_on_remote('192.168.0.110', 'nvidia/File.py', 'content')
            mock_print.assert_called_once_with("Error writing thru SSH : ")
            self.assertEqual(result, None)





if __name__ == '__main__':
    unittest.main()  