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
  availableRoomsSubscription: Subscription | undefined;
  roomDeletedSubscription: Subscription | undefined;
  roomCreatedSubscription: Subscription | undefined;

  constructor(private commandService: CommandService, private socketService: SocketService) { 
    this.availableRooms = [];
    
    this.socketService.stopBatteryCall.asObservable().subscribe((bool: Boolean) => {
      if (!bool) {
        this.socketService.getBatteryLevel('192.168.0.122');
        this.socketService.getBatteryLevel('192.168.0.110');
      }
    })
  }

  ngOnInit(): void {
    this.socketService.getAvailableRooms();

    this.availableRoomsSubscription = this.socketService.getAvailableMissionRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.availableRooms = rooms;
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

  isAvailableRoom(): boolean {
    if (
        this.availableRooms?.length > 0 &&
        this.availableRooms.find((room) => room.robot.ipAddress === this.robot.ipAddress)
    )
        return true;
    else return false;
  }

  ngOnDestroy(): void {
    if (this.availableRoomsSubscription) {
      this.availableRoomsSubscription.unsubscribe();
    }
    if (this.roomDeletedSubscription) {
      this.roomDeletedSubscription.unsubscribe();
    }
    if (this.roomCreatedSubscription) {
      this.roomCreatedSubscription.unsubscribe();
    }
  }
}
