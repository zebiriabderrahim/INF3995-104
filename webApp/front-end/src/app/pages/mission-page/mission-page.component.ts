import { Component, OnInit, OnDestroy } from '@angular/core';
import { MissionRoom, Robot, RobotBatteryInfo } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';
import { Router } from '@angular/router';

@Component({
  selector: 'app-mission-page',
  templateUrl: './mission-page.component.html',
  styleUrls: ['./mission-page.component.css']
})
export class MissionPageComponent implements OnInit, OnDestroy {
  room!: MissionRoom;
  roomSubscription: Subscription | undefined;
  isHostLeavingRoomSubscription: Subscription | undefined;
  availableRoomsSubscription: Subscription | undefined;
  subscription: Subscription | undefined;
  showLogs = false;
  simulation!: boolean;
  allPhysicalRobots!: boolean;
  robots: Robot[];
  isHost!: boolean;
  threeD!: boolean; 
  robotSimulationBatteryLevel: number;
  robotSimulationBatteryLevelRobot2: number;
  allPhysicalRobotsSubscription: Subscription | undefined;
  batteryLevelSimulationSubscription: Subscription | undefined;
  robotBatterySubscription: Subscription | undefined;
  simulationSubscription: Subscription | undefined;


  constructor(public socketService: SocketService, private router: Router) {
    this.router = router;
    this.robots = [];
    this.robotSimulationBatteryLevel = 0;
    this.robotSimulationBatteryLevelRobot2 = 0;
   }

  ngOnInit(): void {
    if (!this.socketService.isConnected()) {
      this.router.navigate(["/home"]);
    }

    this.allPhysicalRobotsSubscription= this.socketService.allPhysicalRobots.asObservable().subscribe((bool: boolean) => {
      this.allPhysicalRobots = bool;
    })

    this.roomSubscription = this.socketService.getMissionRoomInfo().subscribe((roomInfo: MissionRoom) => {
      if (!this.room) {
        this.room = roomInfo;
        if(!this.room.robot.state.includes('simulation')) this.socketService.getLogs(this.room.robot, this.allPhysicalRobots);
      }
    });

    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        this.router.navigate(['/home']);
        this.socketService.isHostLeavingRoom.next(false);
        this.socketService.getAvailableRooms();
      }
    });

    this.simulationSubscription= this.socketService.simulation.asObservable().subscribe((bool) => {
      this.simulation = bool;
    
      if (this.simulation) {
        this.batteryLevelSimulationSubscription= this.socketService.batteryLevelSimulation.asObservable().subscribe((robotBatteryInfo: RobotBatteryInfo) => {
          this.handleRobotBatteryInfo(robotBatteryInfo);
        });
      }
    });
    
    this.robotBatterySubscription= this.socketService.robots.subscribe((robots: Robot[]) => { 
      this.robots = robots
      const robot = robots.find((robot: Robot) => robot.ipAddress === this.room.robot.ipAddress);
      if (robot) {
        this.room.robot.batteryLevel = robot.batteryLevel;
      }
      if(this.room.otherRobots){
        const robot = robots.find((robot: Robot) => robot.ipAddress === this.room.otherRobots?.[0]?.ipAddress);
        if (robot) {
          this.room.otherRobots[0].batteryLevel = robot.batteryLevel;
        }
      }   
    });
  }

  handleRobotBatteryInfo(robotBatteryInfo: RobotBatteryInfo): void {
    if (this.room.robot.ipAddress === "simulation") {

      if (robotBatteryInfo.robotId === "192.168.0.110") this.handleBatteryLevel("Robot 1", robotBatteryInfo.batteryLevel);
      else this.handleBatteryLevel("Robot 2", robotBatteryInfo.batteryLevel);
      
    } else {
      this.handleBatteryLevel(this.room.robot, robotBatteryInfo.batteryLevel);
    }
  }
  
  handleBatteryLevel(robot: string | Robot, batteryLevel: number): void {
    if (typeof robot === 'string' && robot == 'Robot 1') {
      this.robotSimulationBatteryLevel = batteryLevel;
      if (0 < this.robotSimulationBatteryLevel && this.robotSimulationBatteryLevel < 30) {
        this.socketService.returnToBaseSimulation({ "ipAddress":"192.168.0.110",'name': robot });
      }
    } 
    else if (typeof robot === 'string' && robot == 'Robot 2') {
      this.robotSimulationBatteryLevelRobot2 = batteryLevel;
      if (0 < this.robotSimulationBatteryLevelRobot2 && this.robotSimulationBatteryLevelRobot2 <30) {
        this.socketService.returnToBaseSimulation({"ipAddress":"192.168.0.122",'name': robot });
      }
    } 
    else {
      this.robotSimulationBatteryLevel = batteryLevel;
      if (this.robotSimulationBatteryLevel <= 52) {
        this.socketService.returnToBaseSimulation(robot);
      }
    }
  }

  handleStopMissionClick(robot: Robot): void {
    if (this.simulation) {
      this.socketService.terminateSimulationRobot(robot);
      this.socketService.simulation.next(false);
    }
    else if (this.allPhysicalRobots) this.socketService.terminateAllPhysicalRobots(this.robots);
    else this.socketService.stopMission(robot);  
  }

  handleReturnToBase(): void {
    if (this.simulation) {
      this.socketService.returnToBaseSimulation(this.room.robot);
    } else if (this.allPhysicalRobots) this.socketService.returnToBase(this.robots)
    else this.socketService.returnToBase(this.room.robot);
  }


  ngOnDestroy(): void {
    this.isHostLeavingRoomSubscription?.unsubscribe();
    this.roomSubscription?.unsubscribe();
    this.allPhysicalRobotsSubscription?.unsubscribe();
    this.batteryLevelSimulationSubscription?.unsubscribe();
    this.robotBatterySubscription?.unsubscribe();
    this.simulationSubscription?.unsubscribe();

  }
}
