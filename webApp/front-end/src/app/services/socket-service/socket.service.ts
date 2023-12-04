import { Injectable } from '@angular/core';
import { Coordinates, Log, MissionRoom, Robot, RobotBatteryInfo, RobotMarkerInfo } from 'src/app/interfaces/models';
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
  allPhysicalRobots: BehaviorSubject<boolean>;
  robotPos: Subject<RobotMarkerInfo>;
  currentLogs: BehaviorSubject<Log[]>;
  robots: BehaviorSubject<Robot[]>;
  stopBatteryCall: BehaviorSubject<Boolean>;
  stopBatteryCallSimulation: BehaviorSubject<Boolean>;
  rosConnectionError: Subject<boolean>;
  map: BehaviorSubject<number[]>;
  mapHeight: number;
  mapWidth: number;
  batteryLevelSimulation: Subject<RobotBatteryInfo>;
  allSimConnected:BehaviorSubject<boolean>;
  type: string;
  executeReturnToBase: boolean;

  constructor(public clientSocket: ClientSocketService, public router: Router, private communicationService: CommunicationService) {
    this.executeReturnToBase = false;
    this.type = "";
    this.mapHeight = 0;
    this.mapWidth = 0;
    this.roomInfo = new Subject<MissionRoom>();
    this.stopBatteryCall = new BehaviorSubject<Boolean>(false);
    this.stopBatteryCallSimulation = new BehaviorSubject<Boolean>(false);
    this.rosConnectionError = new Subject<boolean>();
    this.availableRooms = new Subject<MissionRoom[]>();
    this.simulationRooms = new Subject<MissionRoom[]>();
    this.isRoomDeleted = new Subject<boolean>();
    this.isRoomCreated = new Subject<boolean>();
    this.isHostLeavingRoom = new Subject<boolean>();
    this.simulation = new BehaviorSubject<boolean>(false);
    this.allPhysicalRobots = new BehaviorSubject<boolean>(false);
    this.robotPos = new Subject<RobotMarkerInfo>();
    this.currentLogs = new BehaviorSubject<Log[]>([]);
    this.batteryLevelSimulation = new Subject<RobotBatteryInfo>();
    this.map = new BehaviorSubject<number[]>([]);
    this.allSimConnected = new BehaviorSubject<boolean>(false);
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

  saveMission(type: string, distance: string) {
    let mission = {
      logs: this.currentLogs.getValue(),
      map:  this.map.getValue(),
      duration: "",
      type: "",
      robots:type,
      distance: distance,
    }
    console.log(mission.distance)
    console.log(type);
    const start = new Date(mission.logs[0].timestamp);
    const end = new Date(mission.logs[mission.logs.length - 1].timestamp);
    const diff = this.calculateDuration(start, end);
    mission.duration = "La mission a duré " + diff.minutes + " minutes et " + diff.secondes + " secondes.";
    if(type.includes("sim")) mission.type = "Mode simulation";
    else  mission.type = "Mode robots physiques";
    console.log(mission);
    this.communicationService.saveMission(mission).subscribe();
  }

  getAvailableRooms() {
    this.clientSocket.send('getAvailableRooms');
  }

  simulateMission() {
    this.clientSocket.send('simulateMission');
    this.router.navigate(["/mission"]);
  }

  simulateMissionRobot(robot: Robot) {
    this.clientSocket.send('simulateMissionRobot', robot);
    this.simulation.next(true);
    this.router.navigate(["/mission"]);
  }

  returnToBase(robot: Robot | Robot[]) {
    this.clientSocket.send('returnToBase', robot);
  }

  returnToBaseSimulation(robot: Robot | {}) {
    if (robot)this.clientSocket.send('returnToBaseSimulation', robot);
  }

  terminateSimulation() {
    this.clientSocket.send('stopSimulation');
  }

  terminateSimulationRobot(robot: Robot) {
    this.clientSocket.send('stopSimulationRobot', robot);
  }

  viewMissionRoom(robot: Robot, isSimulation: boolean) {
    const event= isSimulation ? 'viewMissionRoomSimulation' : 'viewMissionRoom';
    this.clientSocket.send(event, robot);
    this.router.navigate(["/mission"]);
  }

  stopMission(robot: Robot) {
    this.clientSocket.send('stopMission', robot);
  }

  calculateDuration(date1: Date, date2: Date): { minutes: number, secondes: number } {
    const differenceEnMillisecondes = Math.abs(date1.getTime() - date2.getTime());
    const minutes = Math.floor(differenceEnMillisecondes / (1000 * 60));
    const secondes = Math.floor((differenceEnMillisecondes % (1000 * 60)) / 1000);
    return { minutes, secondes };
  }

  getRobotPos(robot: Robot) {
    this.clientSocket.send('getRobotPos', robot.ipAddress);
  }

  getLogs(robot: Robot, allRobots: boolean) {
    if (allRobots) this.clientSocket.send('getLogs', this.robots.getValue());
    else this.clientSocket.send('getLogs', robot);
  }

  getBatteryLevel(robot: Robot) {
    this.clientSocket.send('getBatteryLevel', robot);
  }

  getBatteryLevelSim(robot: Robot) {
    this.clientSocket.send('getBatteryLevelSim', robot);
  }

  launchAllRobots(robots: Robot[]) {
    this.router.navigate(["/mission"]);
    this.clientSocket.send('launchAllRobots', robots);
  }

  terminateAllPhysicalRobots(robots: Robot[]) {
    this.clientSocket.send('stopAllRobots', robots);
  }

  viewMissionRoomAllRobots() {
    this.clientSocket.send('viewAllRobots');
    this.router.navigate(["/mission"]);
  }

  setInitialPosition(name: string, data: any) {
    this.clientSocket.send('setInitialPosition', {'name': name, 'data': data})
  }
  

  handleSocket() {
    this.clientSocket.on('createdMissionRoom', (room: any) => {
      this.currentRoom = room;
      this.currentLogs.next([]);
      this.map.next([]);
      this.roomInfo.next(this.currentRoom);
      this.isRoomCreated.next(true);
      if (room.robot.ipAddress.includes('sim')) {
        this.simulation.next(true);
      }
    });

    this.clientSocket.on('availableRooms', (data: { [key: string]: MissionRoom[] }) => {
      this.simulationRooms.next(data['simulated'])
      this.availableRooms.next(data['rooms']);

    });

    this.clientSocket.on('addedAsViewer', (room: MissionRoom) => {
      console.log("added as viewer");
      this.roomInfo.next(room);
    });
    
    this.clientSocket.on('roomDeleted', (type:string) => {
      this.isRoomDeleted.next(true);
      this.type = type;
      console.log(type);
    })

    this.clientSocket.on('receiveDistanceSim', (distance: string) => {
      console.log(distance);
      if (this.clientSocket.socket.id === this.currentRoom.hostId)  this.saveMission(this.type, distance);
      this.currentLogs.next([]);
      this.map.next([]);
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
      console.log(data)
      this.robotPos.next(data);
    });

    this.clientSocket.on('recieveSimRobot2Pos', (data: RobotMarkerInfo) => {
      this.robotPos.next(data);
    })

    this.clientSocket.on('stoppedSimulation', () => {
      this.simulation.next(false);
    })

    this.clientSocket.on('stopBatteryCall', (bool: Boolean) =>{
      this.stopBatteryCall.next(bool);
    });

    this.clientSocket.on('stopBatteryCallSimulation', (bool: Boolean) =>{
      this.stopBatteryCallSimulation.next(bool);
    });

    this.clientSocket.on('rosConnectionError', () => {
      this.rosConnectionError.next(true);
    })

    this.clientSocket.on('map', (data: number[]) => {
      this.map.next(data);
    });

    this.clientSocket.on('allSimConnected', (bool: boolean) => {
      this.allSimConnected.next(bool);
    });

    this.clientSocket.on('allRobotsConnected', (bool: boolean) => {
      this.allPhysicalRobots.next(bool);
    });

    this.clientSocket.on('receiveBatterySim', (data: RobotBatteryInfo) => {
      this.batteryLevelSimulation.next(data);
    });
 
    this.clientSocket.on('robotBattery', (robot: Robot) => {
      if (robot.batteryLevel < 30 && !this.executeReturnToBase) {
        this.returnToBase(robot); 
        this.executeReturnToBase = true;
      }

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
