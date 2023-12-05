import { Component, Input, OnInit, OnDestroy, OnChanges, SimpleChanges } from '@angular/core';
import { MissionRoom, Robot, RobotBatteryInfo } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';
import { MatDialog } from '@angular/material/dialog';
import { InitialLocationComponent } from '../initial-location/initial-location.component';

@Component({
  selector: 'app-robot',
  templateUrl: './robot.component.html',
  styleUrls: ['./robot.component.css']
})
export class RobotComponent implements OnInit, OnDestroy, OnChanges {
  @Input() robot!: Robot;
  @Input() simulation!: boolean;
  availableRooms: MissionRoom[];
  simulationRooms: MissionRoom[];
  availableRoomsSubscription: Subscription | undefined;
  availableSimRoomsSubscription: Subscription | undefined;
  roomDeletedSubscription: Subscription | undefined;
  roomCreatedSubscription: Subscription | undefined;
  batterySubscription: Subscription | undefined;
  robotSimulationBatteryLevel: number=0;
  stopBatterySimulationSubscription: Subscription | undefined;
  batteryLevelSimulationSubscription: Subscription | undefined;

  constructor(private commandService: CommandService, private socketService: SocketService, private dialog: MatDialog) { 
    this.availableRooms = [];
    this.simulationRooms = [];
  }

  ngOnInit(): void {
    // Fetch available rooms and subscribe to relevant socket service events
    this.socketService.getAvailableRooms();


    this.batterySubscription = this.socketService.stopBatteryCall.asObservable().subscribe((bool: Boolean) => {
      if (!bool) {
        this.socketService.getBatteryLevel(this.robot);
      }
    })

    

    this.stopBatterySimulationSubscription= this.socketService.stopBatteryCallSimulation.asObservable().subscribe((bool: Boolean) => {
      if (!bool) {
        this.socketService.getBatteryLevelSim(this.robot);
      }
    })
     
   
    this.availableRoomsSubscription = this.socketService.getAvailableMissionRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.availableRooms = rooms;
    });

    this.availableSimRoomsSubscription = this.socketService.getAvailableSimulatedRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.simulationRooms = rooms;
    });
 
    this.roomDeletedSubscription = this.socketService.isRoomDeleted.asObservable().subscribe((isDeleted: boolean) => {
      if (isDeleted) {
        this.socketService.getAvailableRooms();
        this.socketService.isRoomDeleted.next(false);
      }
    });
    
    this.roomCreatedSubscription = this.socketService.isRoomCreated.asObservable().subscribe((isCreated: boolean) => {
      if (isCreated) {      
        this.socketService.getAvailableRooms();
        this.socketService.isRoomCreated.next(false);   
      }
    });
    
  }
  
  ngOnChanges(changes: SimpleChanges): void {
    if (this.simulation) {
        this.batteryLevelSimulationSubscription= this.socketService.batteryLevelSimulation.asObservable().subscribe((robotBatteryInfo: RobotBatteryInfo) => {
          if (robotBatteryInfo.robotId === this.robot.ipAddress) this.robotSimulationBatteryLevel = robotBatteryInfo.batteryLevel;
        });
    }
  }

// Function to launch a mission for a robot
launchMission(robot: Robot) {
  this.commandService.createMissionRoom(robot);
}

// Function to identify a robot
identifyRobot(robot: Robot) {
  this.commandService.identifyRobot(robot);
}

// Function to check if all robots are launched in a set of rooms
areAllRobotsLaunched(rooms: MissionRoom[]): boolean {
  for (let room of rooms)
    if (room.robot.ipAddress === 'simulation') {
      return true;
    }
  return false;
}

// Function to view mission details for a robot
viewMission(robot: Robot, isSimulation: boolean) {
  this.commandService.viewMissionRoom(robot, isSimulation);
}

// Function to handle simulation for a robot
handleSimulationRobot(robot: Robot) {
  this.commandService.simulateMissionRobot(robot);
}

// Function to check if a room is available for a robot based on the type ('sim' or undefined)
isAvailableRoom(type: string | undefined): boolean {
  if (type === 'sim') {
      return (
          this.simulationRooms?.some(room => room.robot.ipAddress === this.robot.ipAddress) ||
          false
      );
  } else {
      return (
          this.availableRooms?.some(room => room.robot.ipAddress === this.robot.ipAddress && room.robot.state === 'Active on mission' && !room.otherRobots) ||
          false
      );
  }
}
// Function to check if both robots are used in a set of rooms
bothRobotsUsed(rooms:MissionRoom[]) {
  return (
    rooms.some(room => room.otherRobots && room.otherRobots?.length > 0) ||
    false
  );
}

// Function to check if a robot is in a simulated room
isInSimRoom(robotName: string){
  return this.simulationRooms.some(room => room.robot.name === robotName );
}

// Function to set the initial location for a robot
setInitialLocation(): void {
  // Open dialog for setting initial location
  const dialogRef = this.dialog.open(InitialLocationComponent, {
    width: '400px',
    data: {id: this.robot.name.toLowerCase().replace(' ', '')}
  });

  dialogRef.afterClosed().subscribe(result => {
   if (result) this.socketService.setInitialPosition(this.robot.name, result) 
  });
}

ngOnDestroy(): void {
  // Unsubscribe from all subscriptions to avoid memory leaks
  this.batterySubscription?.unsubscribe();
  this.stopBatterySimulationSubscription?.unsubscribe();
  this.availableRoomsSubscription?.unsubscribe();
  this.availableSimRoomsSubscription?.unsubscribe();
  this.roomDeletedSubscription?.unsubscribe();
  this.roomCreatedSubscription?.unsubscribe();
  this.batteryLevelSimulationSubscription?.unsubscribe();
}

}
