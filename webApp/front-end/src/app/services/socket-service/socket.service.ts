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
  isAlreadyConnected: boolean=false;

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
    /***
     * If the socket is not connected, we try to connect it
     */
    if (!this.clientSocket.isSocketAlive()) {
      this.clientSocket.connect();
    }
  }

  isConnected(): boolean {
    /***
     * Returns true if the socket is connected
     */
    return this.clientSocket.isSocketAlive();
  }

  createMissionRoom(robot: Robot) {
    /***
     * Creates a mission room with the given robot
     * @param robot
     * @returns void
     * @emits createdMissionRoom
     */
    this.clientSocket.send('createMissionRoom', robot);
    this.router.navigate(["/mission"]);
  }

  navigate(path: String) {
    /***
     * Navigates to the given path
     * @param path
     * @returns void
     */
    this.router.navigate([path]);
  }

  getMissionRoomInfo(): Observable<MissionRoom> {
    /***
     * Returns the current mission room info
     * @returns Observable<MissionRoom>
     */
    return this.roomInfo.asObservable();
  }

  getAvailableMissionRoomsInfo(): Observable<MissionRoom[]> {
    /***
     * Returns the available mission rooms info
     * @returns Observable<MissionRoom[]>
     */
    return this.availableRooms.asObservable();
  }

  getAvailableSimulatedRoomsInfo(): Observable<MissionRoom[]> {
    /***
     * Returns the available simulated mission rooms info
     * @returns Observable<MissionRoom[]>
     */
    return this.simulationRooms.asObservable();
  }

  saveMission(type: string, distance: string) {
    /***
     * Saves the mission in the database
     * @param type: type of the mission
     * @param distance: distance travelled by the robot
     *
     * @returns void
     *
     */
    let mission = {
      logs: this.currentLogs.getValue(),
      map:  this.map.getValue(),
      duration: "",
      type: "",
      robots:type,
      distance: distance,
    }
    const start = new Date(mission.logs[0].timestamp);
    const end = new Date(mission.logs[mission.logs.length - 1].timestamp);
    const diff = this.calculateDuration(start, end);
    mission.duration = "La mission a duré " + diff.minutes + " minutes et " + diff.secondes + " secondes.";
    if(type.includes("sim")) mission.type = "Mode simulation";
    else  mission.type = "Mode robots physiques";

    this.communicationService.saveMission(mission).subscribe();
  }

  getAvailableRooms() {
    /***
     * Returns the available rooms
     * @returns void
     * @emits availableRooms
     */
    this.clientSocket.send('getAvailableRooms');
  }

  simulateMission() {
    /***
     * Simulates a mission with all robots and navigates to the mission page
     * @returns void
     * @emits simulateMission
     */
    this.clientSocket.send('simulateMission');
    this.router.navigate(["/mission"]);
  }

  simulateMissionRobot(robot: Robot) {
    /***
     * Simulates a mission with the given robot and navigates to the mission page
     * @param robot: robot to simulate
     * @returns void
     * @emits simulateMissionRobot
     */
    this.clientSocket.send('simulateMissionRobot', robot);
    this.simulation.next(true);
    this.router.navigate(["/mission"]);
  }

  returnToBase(robot: Robot | Robot[]) {
    /***
     * Returns the robot to its initial position
     * @param robot: robot to return to base or robots to return to base
     * @returns void
     * @emits returnToBase
     */
    this.clientSocket.send('returnToBase', robot);
  }

  returnToBaseSimulation(robot: Robot | {}) {
    /***
     * Returns the robot to its initial position in simulation mode
     * @param robot: robot to return to base or robots to return to base
     * @returns void
     * @emits returnToBaseSimulation
     */
    if (robot)this.clientSocket.send('returnToBaseSimulation', robot);
  }

  terminateSimulation() {
    /***
     * Terminates the simulation
     * @returns void
     * @emits stopSimulation
     */
    this.clientSocket.send('stopSimulation');
  }

  terminateSimulationRobot(robot: Robot) {
    /***
     * Terminates the simulation of the given robot
     * @param robot: robot to terminate the simulation
     * @returns void
     * @emits stopSimulationRobot
     */
    this.clientSocket.send('stopSimulationRobot', robot);
  }

  viewMissionRoom(robot: Robot, isSimulation: boolean) {
    /***
     * Views the mission room with the given robot
     * @param robot: robot to view the mission room
     * @param isSimulation: true if the robot is in simulation, false otherwise
     * @returns void
     * @emits viewMissionRoom or viewMissionRoomSimulation depending on the value of isSimulation
     */
    const event= isSimulation ? 'viewMissionRoomSimulation' : 'viewMissionRoom';
    this.clientSocket.send(event, robot);
    this.router.navigate(["/mission"]);
  }

  stopMission(robot: Robot) {
    /***
     * Stops the mission of the given robot
     * @param robot: robot to stop the mission
     * @returns void
     * @emits stopMission
     */
    this.clientSocket.send('stopMission', robot);
  }

  calculateDuration(date1: Date, date2: Date): { minutes: number, secondes: number } {
    /***
     * Calculates the duration between start and end of the mission
     * @param date1: start date
     * @param date2: end date
     * @returns { minutes: number, secondes: number }
     */
    const differenceEnMillisecondes = Math.abs(date1.getTime() - date2.getTime());
    const minutes = Math.floor(differenceEnMillisecondes / (1000 * 60));
    const secondes = Math.floor((differenceEnMillisecondes % (1000 * 60)) / 1000);
    return { minutes, secondes };
  }

  getRobotPos(robot: Robot) {
    /***
     * Gets the position of the given robot
     * @param robot: robot to get the position
     * @returns void
     * @emits getRobotPos
     */
    this.clientSocket.send('getRobotPos', robot.ipAddress);
  }

  getLogs(robot: Robot, allRobots: boolean) {
    /***
     * Gets the logs of the given robot or all robots
     * @param robot: robot to get the logs
     * @param allRobots: true if we want to get the logs of all robots, false otherwise
     * @returns void
     * @emits getLogs
     */
    if (allRobots) this.clientSocket.send('getLogs', this.robots.getValue());
    else this.clientSocket.send('getLogs', robot);
  }

  getBatteryLevel(robot: Robot) {
    /***
     * Start the battery subscription of the given robot
     * @param robot: robot to start the battery subscription
     * @returns void
     * @emits getBatteryLevel
     */
    this.clientSocket.send('getBatteryLevel', robot);
  }

  getBatteryLevelSim(robot: Robot) {
    /***
     * Start the battery subscription of the given robot in simulation mode
     * @param robot: robot to start the battery subscription
     * @returns void
     * @emits getBatteryLevelSim
     */
    this.clientSocket.send('getBatteryLevelSim', robot);
  }

  launchAllRobots(robots: Robot[]) {
    /***
     * Launches all robots and navigates to the mission page
     * @param robots: robots to launch
     * @returns void
     * @emits launchAllRobots
     */
    this.router.navigate(["/mission"]);
    this.clientSocket.send('launchAllRobots', robots);
  }

  terminateAllPhysicalRobots(robots: Robot[]) {
    /***
     * Terminates all physical robots in mission
     * @param robots: robots to terminate
     * @returns void
     * @emits terminateAllPhysicalRobots
     */
    this.clientSocket.send('stopAllRobots', robots);
  }

  viewMissionRoomAllRobots() {
    /***
     * Views the mission room with all robots and navigates to the mission page
     * @returns void
     * @emits viewMissionRoomAllRobots
     */
    this.clientSocket.send('viewAllRobots');
    this.router.navigate(["/mission"]);
  }

  setInitialPosition(name: string, data: any) {
    /***
     * Sets the initial position of the given robot
     * @param name: name of the robot
     * @param data: initial position of the robot (x, y)
     * @returns void
     * @emits setInitialPosition
     */
    this.clientSocket.send('setInitialPosition', {'name': name, 'data': data})
  }


  handleSocket() {
    /***
     * Handles the socket events
     * @returns void
     */
    this.clientSocket.on('roomCreated', (room:any) => {
      /***
       * When a room is created, we set the current room to the created room and we set the isRoomCreated to true
       */
      this.isRoomCreated.next(true);
      if (room.robot.ipAddress.includes('sim')) {
        this.simulation.next(true);
      }
      this.isAlreadyConnected = true;
    });

    this.clientSocket.on('createdMissionRoom', (room: any) => {
      /***
       * When a mission room is created, we set the current room to the created room and we reset the logs and the map
       */
        this.currentRoom = room;
        this.currentLogs.next([]);
        this.map.next([]);
        this.roomInfo.next(this.currentRoom);

    });

    this.clientSocket.on('availableRooms', (data: { [key: string]: MissionRoom[] }) => {
      /***
       * When the available rooms are received, we set the available rooms and the simulated rooms
       */
      this.simulationRooms.next(data['simulated'])
      this.availableRooms.next(data['rooms']);

    });

    this.clientSocket.on('addedAsViewer', (room: MissionRoom) => {
      /***
       * When the user is added as viewer, we set the current room
       */
      this.roomInfo.next(room);
    });

    this.clientSocket.on('roomDeleted', (type:string) => {
      /***
       * When the room is deleted, we set the isRoomDeleted to true and we set the type of the mission
       */
      this.isRoomDeleted.next(true);
      this.type = type;
    })

    this.clientSocket.on('receiveDistanceSim', (distance: string) => {
      /***
       * When the distance is received, we save the mission and we reset the logs and the map
       */
      if (this.currentRoom && this.clientSocket.socket.id === this.currentRoom.hostId)  this.saveMission(this.type, distance);
      this.currentLogs.next([]);
      this.map.next([]);
    })

    this.clientSocket.on('hostLeftRoom', () => {
      /***
       * When the host leaves the room, we set the isHostLeavingRoom to true
       */
      this.isHostLeavingRoom.next(true);
    });

    this.clientSocket.on('log', (log: Log) => {
      /***
       * When a log is received, we add it to the current logs of the mission
       */
      const currentLogs = this.currentLogs.getValue();
      currentLogs.push(log);
      this.currentLogs.next(currentLogs);
    });

    this.clientSocket.on('recieveSimRobotPos', (data: RobotMarkerInfo) => {
      /***
       * When the position of the robot 1 is received, we set the robot position for the map to draw it
       */
      this.robotPos.next(data);
    });

    this.clientSocket.on('recieveSimRobot2Pos', (data: RobotMarkerInfo) => {
      /***
       * When the position of the robot 2 is received, we set the robot position for the map to draw it
       */
      this.robotPos.next(data);
    })

    this.clientSocket.on('stoppedSimulation', () => {
      /***
       * When the simulation is stopped, we set the simulation boolean to false
       */
      this.simulation.next(false);
    })

    this.clientSocket.on('stopBatteryCall', (bool: Boolean) =>{
      /***
       * When the battery call is stopped, we set the stopBatteryCall boolean to true
       */
      this.stopBatteryCall.next(bool);
    });

    this.clientSocket.on('stopBatteryCallSimulation', (bool: Boolean) =>{
      /***
       * When the battery call in simulation is stopped, we set the stopBatteryCallSimulation boolean to true to stop updating the battery level
       */
      this.stopBatteryCallSimulation.next(bool);
    });

    this.clientSocket.on('rosConnectionError', () => {
      /***
       * When the ROS connection is lost, we set the rosConnectionError boolean to true
       */
      this.rosConnectionError.next(true);
    })

    this.clientSocket.on('map', (data: number[]) => {
      /***
       * When the map is received, we set the map used in the mission page
       */
      this.map.next(data);
    });

    this.clientSocket.on('allSimConnected', (bool: boolean) => {
      /***
       * When all simulated robots are launched, we set the allSimConnected boolean to true to get everything from the two robots
       */
      this.allSimConnected.next(bool);
    });

    this.clientSocket.on('allRobotsConnected', (bool: boolean) => {
      /***
       * When all physical robots are launched, we set the allPhysicalRobots boolean to true to get everything from the two robots
       */
      this.allPhysicalRobots.next(bool);
    });

    this.clientSocket.on('receiveBatterySim', (data: RobotBatteryInfo) => {
      /***
       * When the battery level of the robot in simulation is received, we set the battery level
       */
      this.batteryLevelSimulation.next(data);
    });

    this.clientSocket.on('robotBattery', (robot: Robot) => {
      /***
       * When the battery level of the robot is received, we set the battery level and we return the robot to base if the battery level is less than 30
       * If the robot is not in the robots list, we add it to the list
       * If the robot is in the robots list, we update its battery level and its state
       */

      if(this.robots.getValue().length === 0 || !this.robots.value.find((r: Robot) => r.ipAddress === robot.ipAddress)){
        this.robots.next([...this.robots.value, robot]);
      }
      else{
        this.robots.next(this.robots.value.map((robot_old: Robot) => {
          if (robot.batteryLevel < 30 && !this.executeReturnToBase) {
            this.returnToBase(this.currentRoom.robot);
            this.executeReturnToBase = true;
          }

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
