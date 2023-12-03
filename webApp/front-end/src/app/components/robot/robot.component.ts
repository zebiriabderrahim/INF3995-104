import { Component, Input, OnInit, OnDestroy, OnChanges, SimpleChanges, ChangeDetectorRef } from '@angular/core';
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

  launchMission(robot: Robot) {
    this.commandService.createMissionRoom(robot);
  }

  identifyRobot(robot: Robot) {
    this.commandService.identifyRobot(robot);
  }

  isAllRobotsLaunched(rooms: MissionRoom[]): boolean {
    for (let room of rooms)  
      if (room.robot.ipAddress === 'simulation'){
        return true;
      }
    return false;
  }

  viewMission(robot: Robot, isSimulation: boolean) {
    this.commandService.viewMissionRoom(robot,isSimulation);
  }

  handleSimulationRobot(robot: Robot) {
    this.commandService.simulateMissionRobot(robot);
  }

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

  bothRobotsUsed(rooms:MissionRoom[]) {
    return (
      rooms.some(room => room.otherRobots && room.otherRobots?.length > 0) ||
      false
    );
  }

  isInSimRoom(robotName: string){
    return this.simulationRooms.some(room => room.robot.name === robotName );
  }


  setInitialLocation(): void {
    const dialogRef = this.dialog.open(InitialLocationComponent, {
      width: '400px',
      data: {id: this.robot.name.toLowerCase().replace(' ', '')}
    });

    dialogRef.afterClosed().subscribe(result => {
     if (result) this.socketService.setInitialPosition(this.robot.name, result) 
    });
  }

  ngOnDestroy(): void {
    this.batterySubscription?.unsubscribe();
    this.stopBatterySimulationSubscription?.unsubscribe();
    this.availableRoomsSubscription?.unsubscribe();
    this.availableSimRoomsSubscription?.unsubscribe();
    this.roomDeletedSubscription?.unsubscribe();
    this.roomCreatedSubscription?.unsubscribe();
    this.batteryLevelSimulationSubscription?.unsubscribe();

  }
}
