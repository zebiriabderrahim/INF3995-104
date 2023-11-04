import { Injectable } from '@angular/core';
import { Coordinates, Log, MissionRoom, Robot, RobotMarkerInfo } from 'src/app/interfaces/models';
import { Router } from '@angular/router';
import { ClientSocketService } from '../client-socket/client-socket.service';
import { BehaviorSubject, Observable, Subject } from 'rxjs';
import { CommunicationService } from '../communication-service/communication.service';

@Injectable({
  providedIn: 'root'
})
export class SocketService {
  currentRoom!: MissionRoom;
  roomInfo: Subject<MissionRoom>;
  availableRooms: Subject<MissionRoom[]>;
  simulationRooms: Subject<MissionRoom[]>;
  isRoomDeleted: Subject<boolean>;
  isRoomCreated: Subject<boolean>;
  isHostLeavingRoom: Subject<boolean>;
  simulation: BehaviorSubject<boolean>;
  robotPos: Subject<RobotMarkerInfo>;
  currentLogs: BehaviorSubject<Log[]>;
  robots: BehaviorSubject<Robot[]>;
  stopBatteryCall: BehaviorSubject<Boolean>;
  rosConnectionError: Subject<boolean>;
  map: Subject<any>;

  constructor(public clientSocket: ClientSocketService, public router: Router, private communicationService: CommunicationService) {
    this.roomInfo = new Subject<MissionRoom>();
    this.stopBatteryCall = new BehaviorSubject<Boolean>(false);
    this.rosConnectionError = new Subject<boolean>();
    this.availableRooms = new Subject<MissionRoom[]>();
    this.simulationRooms = new Subject<MissionRoom[]>();
    this.isRoomDeleted = new Subject<boolean>();
    this.isRoomCreated = new Subject<boolean>();
    this.isHostLeavingRoom = new Subject<boolean>();  
    this.simulation = new BehaviorSubject<boolean>(false);
    this.robotPos = new Subject<RobotMarkerInfo>();
    this.currentLogs = new BehaviorSubject<Log[]>([]);
    this.map = new Subject<any>();
    this.robots = new BehaviorSubject<Robot[]>([
      {
        name: 'Robot 1',
        ipAddress: '192.168.0.110',
        state: 'Off',
        batteryLevel: 0,
      },
      {
        name: 'Robot 2',
        ipAddress: '192.168.0.122',
        state: 'Off',
        batteryLevel: 0,
      },
    ]);
  }

  connect() {
    if (!this.clientSocket.isSocketAlive()) {
      this.clientSocket.connect();
    }
  }

  isConnected(): boolean {
    return this.clientSocket.isSocketAlive();
  }

  createMissionRoom(robot: Robot) {
    this.clientSocket.send('createMissionRoom', robot);
    this.router.navigate(["/mission"]);
  }

  navigate(path: String) {
    this.router.navigate([path]);
  }

  getMissionRoomInfo(): Observable<MissionRoom> {
    return this.roomInfo.asObservable();
  }

  getAvailableMissionRoomsInfo(): Observable<MissionRoom[]> {
    return this.availableRooms.asObservable();
  }

  getAvailableSimulatedRoomsInfo(): Observable<MissionRoom[]> {
    return this.simulationRooms.asObservable();
  }

  saveMission() {
    let mission = {
      logs: this.currentLogs.getValue()
    }
    this.communicationService.saveMission(mission).subscribe();
  }

  getAvailableRooms() {
    this.clientSocket.send('getAvailableRooms');
  }

  simulateMission() {
    this.clientSocket.send('simulateMission');
    this.simulation.next(true);
    this.router.navigate(["/mission"]);
  }

  simulateMissionRobot(robot: Robot) {
    this.clientSocket.send('simulateMissionRobot', robot);
    this.simulation.next(true);
    this.router.navigate(["/mission"]);
  }

  terminateSimulation() {
    this.clientSocket.send('stopSimulation');
  }

  terminateSimulationRobot(robot: Robot) {
    this.clientSocket.send('stopSimulationRobot', robot);
  }

  viewMissionRoom(robot: Robot) {
    this.clientSocket.send('viewMissionRoom', robot);
    this.router.navigate(["/mission"]);
  }

  stopMission(robot: Robot) {
    this.clientSocket.send('stopMission', robot);
  }

  getRobotPos(robot: Robot) {
    this.clientSocket.send('getRobotPos', robot.ipAddress);
  }
  getLogs(robot: Robot) {
    this.clientSocket.send('getLogs', robot.ipAddress);
  }
  
  getBatteryLevel(robotIp: String) {
    this.clientSocket.send('getBatteryLevel', robotIp);
  }

  handleSocket() {
    this.clientSocket.on('createdMissionRoom', (room: any) => {
      this.currentRoom = room;
      this.currentLogs.next([]);
      this.map.next(undefined);
      this.roomInfo.next(this.currentRoom);
      this.isRoomCreated.next(true);
    });

    this.clientSocket.on('availableRooms', (data: { [key: string]: MissionRoom[] }) => { 
      this.simulationRooms.next(data['simulated'])
      this.availableRooms.next(data['rooms']);
    });

    this.clientSocket.on('addedAsViewer', (room: MissionRoom) => {
      this.roomInfo.next(room);
    });
    
    this.clientSocket.on('roomDeleted', () => {
      this.isRoomDeleted.next(true);
      this.saveMission();
      this.currentLogs.next([]);
    })

    this.clientSocket.on('hostLeftRoom', () => {
      this.isHostLeavingRoom.next(true);
    });

    this.clientSocket.on('log', (log: Log) => {
      const currentLogs = this.currentLogs.getValue();
      currentLogs.push(log);
      this.currentLogs.next(currentLogs);
    }); 

    this.clientSocket.on('recieveSimRobotPos', (data: RobotMarkerInfo) => {
      this.robotPos.next(data);
    });

    this.clientSocket.on('stoppedSimulation', () => {
      this.simulation.next(false);
    })
    
    this.clientSocket.on('stopBatteryCall', (bool: Boolean) =>{
      this.stopBatteryCall.next(bool);
    });

    this.clientSocket.on('rosConnectionError', () => {
      this.rosConnectionError.next(true);
    })

    this.clientSocket.on('map', (map: any) => {
      this.map.next(map);
      console.log(map);
    });

    this.clientSocket.on('robotBattery', (robot: Robot) => {
      if(this.robots.getValue().length === 0 || !this.robots.value.find((r: Robot) => r.ipAddress === robot.ipAddress)){
        this.robots.next([...this.robots.value, robot]);
      }
      else{
        this.robots.next(this.robots.value.map((robot_old: Robot) => {
          if(robot_old.ipAddress === robot.ipAddress && robot_old.batteryLevel != robot.batteryLevel){
            return {
              ...robot_old,
              state: "On",
              batteryLevel: robot.batteryLevel
            }
          }
          else{
            return robot_old;
          }
        }));
      }
    });
  }
}
