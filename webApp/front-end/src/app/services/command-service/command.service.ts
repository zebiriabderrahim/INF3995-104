import { Injectable } from '@angular/core';
import { CommunicationService } from '../communication-service/communication.service';
import { SocketService } from '../socket-service/socket.service';
import { Robot } from 'src/app/interfaces/models';

@Injectable({
  providedIn: 'root'
})
export class CommandService {
 
  constructor(private communicationService: CommunicationService, public socketService: SocketService) { }
  
  getRobots(){
    return this.communicationService.getRobots();
  }

  getMissions(){
    return this.communicationService.getMissions();
  }

  identifyRobot(robot: Robot){ 
    this.communicationService.identifyRobot(robot).subscribe();
  }

  launchMission(robot: Robot){ 
    this.communicationService.launchMission(robot).subscribe();
  }


  createMissionRoom(robot: Robot) { 
    this.launchMission(robot);
    this.socketService.createMissionRoom(robot);
  }

  viewMissionRoom(robot: Robot) {
    this.socketService.viewMissionRoom(robot);
  }

  simulateMission() {
    this.socketService.simulateMission();
  }

  simulateMissionRobot(robot: Robot) {
    this.socketService.simulateMissionRobot(robot);
  }

  terminateSimulation() {
    this.communicationService.terminateSimulation().subscribe();
  }

  getRosConnection(robot: Robot) {
    return this.communicationService.getRosConnection(robot);
  }
}
