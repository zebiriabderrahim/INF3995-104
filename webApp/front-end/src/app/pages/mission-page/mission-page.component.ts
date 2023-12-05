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
  // Properties related to the mission room and its information
  room!: MissionRoom;
  roomSubscription: Subscription | undefined;
  isHostLeavingRoomSubscription: Subscription | undefined;
  availableRoomsSubscription: Subscription | undefined;
  subscription: Subscription | undefined;
  
  // Properties related to the display and behavior of the component
  showLogs = false;
  simulation!: boolean;
  allPhysicalRobots!: boolean;
  robots: Robot[];
  isHost!: boolean;
  threeD!: boolean; 
  
  // Properties related to battery levels in the simulation
  robotSimulationBatteryLevel: number;
  robotSimulationBatteryLevelRobot2: number;
  allPhysicalRobotsSubscription: Subscription | undefined;
  batteryLevelSimulationSubscription: Subscription | undefined;
  robotBatterySubscription: Subscription | undefined;
  simulationSubscription: Subscription | undefined;

  constructor(public socketService: SocketService, private router: Router) {
    // Initialize component properties
    this.router = router;
    this.robots = [];
    this.robotSimulationBatteryLevel = 0;
    this.robotSimulationBatteryLevelRobot2 = 0;
  }

  ngOnInit(): void {
    // Check if the socket is connected, otherwise redirect to the home page
    if (!this.socketService.isConnected()) {
      this.router.navigate(["/home"]);
    }

    // Subscribe to updates about the availability of physical robots
    this.allPhysicalRobotsSubscription = this.socketService.allPhysicalRobots.asObservable().subscribe((bool: boolean) => {
      this.allPhysicalRobots = bool;
    })

    // Subscribe to updates about the mission room information
    this.roomSubscription = this.socketService.getMissionRoomInfo().subscribe((roomInfo: MissionRoom) => {
      if (!this.room) {
        this.room = roomInfo;
        // Get logs if the robot is not in simulation mode
        if (!this.room.robot.state.includes('simulation')) this.socketService.getLogs(this.room.robot, this.allPhysicalRobots);
      }
    });

    // Subscribe to updates about the host leaving the room
    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        // Navigate to the home page, reset flags, and get available rooms
        this.router.navigate(['/home']);
        this.socketService.isHostLeavingRoom.next(false);
        this.socketService.getAvailableRooms();
      }
    });

    // Subscribe to updates about the simulation status
    this.simulationSubscription = this.socketService.simulation.asObservable().subscribe((bool) => {
      this.simulation = bool;

      // If in simulation mode, subscribe to battery level updates
      if (this.simulation) {
        this.batteryLevelSimulationSubscription = this.socketService.batteryLevelSimulation.asObservable().subscribe((robotBatteryInfo: RobotBatteryInfo) => {
          this.handleRobotBatteryInfo(robotBatteryInfo);
        });
      }
    });

    // Subscribe to updates about the battery levels of physical robots
    this.robotBatterySubscription = this.socketService.robots.subscribe((robots: Robot[]) => {
      this.robots = robots
      const robot = robots.find((robot: Robot) => robot.ipAddress === this.room.robot.ipAddress);
      if (robot) {
        this.room.robot.batteryLevel = robot.batteryLevel;
      }
      if (this.room.otherRobots) {
        const robot = robots.find((robot: Robot) => robot.ipAddress === this.room.otherRobots?.[0]?.ipAddress);
        if (robot) {
          this.room.otherRobots[0].batteryLevel = robot.batteryLevel;
        }
      }
    });
  }

  // Handle updates in robot battery information 
  handleRobotBatteryInfo(robotBatteryInfo: RobotBatteryInfo): void {
    if (this.room.robot.ipAddress === "simulation") {
      // Determine which simulated robot's battery level to handle
      if (robotBatteryInfo.robotId === "192.168.0.110") this.handleBatteryLevel("Robot 1", robotBatteryInfo.batteryLevel);
      else this.handleBatteryLevel("Robot 2", robotBatteryInfo.batteryLevel);
    } else {
      // Handle battery level update for the specific robot in the room
      if (this.room.robot.ipAddress === robotBatteryInfo.robotId) this.handleBatteryLevel(this.room.robot, robotBatteryInfo.batteryLevel);
    }
  }

  // Handle battery level updates for robots
  handleBatteryLevel(robot: string | Robot, batteryLevel: number): void {
    if (typeof robot === 'string' && robot == 'Robot 1') {
      this.robotSimulationBatteryLevel = batteryLevel;
      // If battery level is low, simulate returning to base
      if (0 < this.robotSimulationBatteryLevel && this.robotSimulationBatteryLevel < 30) {
        this.socketService.returnToBaseSimulation({ "ipAddress": "192.168.0.110", 'name': robot });
      }
    } else if (typeof robot === 'string' && robot == 'Robot 2') {
      this.robotSimulationBatteryLevelRobot2 = batteryLevel;
      // If battery level is low, simulate returning to base
      if (0 < this.robotSimulationBatteryLevelRobot2 && this.robotSimulationBatteryLevelRobot2 < 30) {
        this.socketService.returnToBaseSimulation({ "ipAddress": "192.168.0.122", 'name': robot });
      }
    } else {
      // Handle battery level update for the specific robot in the room
      this.robotSimulationBatteryLevel = batteryLevel;
      // If battery level is low, simulate returning to base
      if (0 < this.robotSimulationBatteryLevel && this.robotSimulationBatteryLevel < 30) {
        this.socketService.returnToBaseSimulation(robot);
      }
    }
  }

  // Handle the click event to stop the mission for a specific robot
  handleStopMissionClick(robot: Robot): void {
    if (this.simulation) {
      // Terminate simulation for the specified robot and reset simulation flag
      this.socketService.terminateSimulationRobot(robot);
      this.socketService.simulation.next(false);
    } else if (this.allPhysicalRobots) {
      // Terminate all physical robots
      this.socketService.terminateAllPhysicalRobots(this.robots);
    } else {
      // Stop the mission for the specified robot
      this.socketService.stopMission(robot);
    }
  }

  // Handle the click event to return the robot to base
  handleReturnToBase(): void {
    if (this.simulation) {
      // Simulate returning the robot to base in the simulation
      this.socketService.returnToBaseSimulation(this.room.robot);
    } else if (this.allPhysicalRobots) {
      // Return all physical robots to base
      this.socketService.returnToBase(this.robots)
    } else {
      // Return the specific robot in the room to base
      this.socketService.returnToBase(this.room.robot);
    }
  }

  ngOnDestroy(): void {
    // Unsubscribe from all subscriptions to prevent memory leaks
    this.isHostLeavingRoomSubscription?.unsubscribe();
    this.roomSubscription?.unsubscribe();
    this.allPhysicalRobotsSubscription?.unsubscribe();
    this.batteryLevelSimulationSubscription?.unsubscribe();
    this.robotBatterySubscription?.unsubscribe();
    this.simulationSubscription?.unsubscribe();
  }
}
