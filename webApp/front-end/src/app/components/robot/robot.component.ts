import { Component, Input, OnInit, OnDestroy, OnChanges, SimpleChanges } from '@angular/core';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';

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

  constructor(private commandService: CommandService, private socketService: SocketService) { 
    this.availableRooms = [];
    this.simulationRooms = [];
    

  }

  ngOnInit(): void {
    this.socketService.getAvailableRooms();
    
    this.batterySubscription=this.socketService.stopBatteryCall.asObservable().subscribe((bool: Boolean) => {
      if (!bool) {
        this.socketService.getBatteryLevel(this.robot.ipAddress);
        // this.socketService.getBatteryLevel('192.168.0.110');
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

  ngOnChanges(changes: SimpleChanges): void {}

  launchMission(robot: Robot) {
    this.commandService.createMissionRoom(robot);
  }

  indentifyRobot(robot: Robot) {
    this.commandService.identifyRobot(robot);
  }

  viewMission(robot: Robot) {
    this.commandService.viewMissionRoom(robot);
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
            this.availableRooms?.some(room => room.robot.ipAddress === this.robot.ipAddress && room.robot.state === 'Active on mission') ||
            false
        );
    }
  }

  isInSimRoom(robotName: string){
    return this.simulationRooms.some(room => room.robot.name === this.robot.name);
  }

  ngOnDestroy(): void {
    this.batterySubscription?.unsubscribe()
    // if (this.availableRoomsSubscription) {
    //   this.availableRoomsSubscription.unsubscribe();
    // }
    // if (this.availableSimRoomsSubscription) {
    //   this.availableSimRoomsSubscription.unsubscribe();
    // }
    // if (this.roomDeletedSubscription) {
    //   this.roomDeletedSubscription.unsubscribe();
    // }
    // if (this.roomCreatedSubscription) {
    //   this.roomCreatedSubscription.unsubscribe();
    // }
  }
}
