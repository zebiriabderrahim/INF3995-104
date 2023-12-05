import { Injectable } from '@angular/core';
import { CommunicationService } from '../communication-service/communication.service';
import { SocketService } from '../socket-service/socket.service';
import { Robot } from 'src/app/interfaces/models';

@Injectable({
  providedIn: 'root'
})
export class CommandService {

  constructor(private communicationService: CommunicationService, public socketService: SocketService) { }

  getMissions(){
    /***
     * GET request to get all missions from database
     */
    return this.communicationService.getMissions();
  }

  getMissionMap(missionName: string){
    /***
     * GET request to get the mission map
     * @param missionName: mission name of the map
     */
    return this.communicationService.getMissionMap(missionName);
  }

  identifyRobot(robot: Robot){
    /***
     * GET request to identify specific robot with a sound
     * @param robot: Robot
     */
    this.communicationService.identifyRobot(robot).subscribe();
  }

  createMissionRoom(robot: Robot) {
    /***
     * Create a mission room for a robot
     * @param robot: Robot
     */
    this.socketService.createMissionRoom(robot);
  }

  viewMissionRoom(robot: Robot, isSimulation: boolean) {
    /***
     * View the mission room for a robot
     * @param robot: Robot
     * @param isSimulation: if it is a simulation room
     */
    this.socketService.viewMissionRoom(robot,isSimulation);
  }

  simulateMission() {
    /***
     * Simulate a mission with all robots
     */
    this.socketService.simulateMission();
  }

  simulateMissionRobot(robot: Robot) {
    /***
     * Simulate a mission with a specific robot
     * @param robot: Robot
     */
    this.socketService.simulateMissionRobot(robot);
  }

  launchAllRobots(robots: Robot[]) {
    /***
     * Launch all physical robots
     * @param robots: Robot[]
     */
    this.socketService.launchAllRobots(robots);
  }

  viewMission() {
    /***
     * View the mission room of all robots
     */
    this.socketService.viewMissionRoomAllRobots();
  }

  getRobotFiles(password: string) {
    /***
     * GET request to get the robot files for the code editor
     * @param password: password of the robot
     */
    return this.communicationService.getRobotFiles(password);
  }

  saveRobotFiles(password: string, files: any) {
    /***
     * POST request to save the robot files of the code editor
     * @param password: password of the robot
     * @param files: files to be saved
     */
    return this.communicationService.saveRobotFiles(password, files);
  }

}
