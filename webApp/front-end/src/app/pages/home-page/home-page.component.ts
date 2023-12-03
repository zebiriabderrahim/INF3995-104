import { ChangeDetectorRef, Component, OnChanges, OnInit, SimpleChanges } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { Subscription } from 'rxjs';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';
import { MissionRoom, Robot, RobotBatteryInfo } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';

@Component({
  selector: 'app-home-page',
  templateUrl: './home-page.component.html',
  styleUrls: ['./home-page.component.css'],
})
export class HomePageComponent implements OnInit {
  title = 'front-end';
  simulationStatus: boolean;
  simulationRooms: MissionRoom[];
  availableRooms: MissionRoom[];
  robots: Robot[];
  availableSimRoomsSubscription: Subscription | undefined;
  rosConnectionErrorSubscription: Subscription | undefined;
  availableRoomsSubscription: Subscription | undefined;
  robotSubscription: Subscription | undefined;
  allSimConnectedSubscription: Subscription | undefined;
  enableSimulation!: boolean;
  areRobotsOn: boolean = false;

  constructor(
    private commandService: CommandService,
    private dialog: MatDialog,
    private socketService: SocketService,
  ) {
    this.simulationRooms = [];
    this.availableRooms = [];
    this.robots = [];
    this.simulationStatus = false;
  }


  ngOnInit(): void {
    this.socketService.getAvailableRooms()
    this.allSimConnectedSubscription= this.socketService.allSimConnected.subscribe((allSimConnected: boolean) => {
        this.enableSimulation = allSimConnected;
    });

    this.robotSubscription= this.socketService.robots.subscribe((robots: Robot[]) => {
      this.robots = robots;
      if (robots[0].state === 'On' && robots[1].state === 'On') {
        this.areRobotsOn = true;
      }
    });


    this.rosConnectionErrorSubscription = this.socketService.rosConnectionError.subscribe(rosConnectionError => {
      if (rosConnectionError) {
        this.openErrorDialog({
          title: 'Error',
          message: 'Ros connection not established',
          close: 'close',
        });
        this.socketService.rosConnectionError.next(false);
      }

      this.socketService.navigate('/home')
    })

    this.availableSimRoomsSubscription = this.socketService.getAvailableSimulatedRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.simulationRooms = rooms;
    });

    this.availableRoomsSubscription = this.socketService.getAvailableMissionRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.availableRooms = rooms;
    });

  }

  handleSimulation() {
    this.commandService.simulateMission();
  }

  handleRobots() {
    this.commandService.launchAllRobots(this.robots);
  }

  viewMission() {
    this.commandService.viewMission();
  }

  bothRobotsUsed(rooms:MissionRoom[]) {
    return (
      rooms.some(room => room.otherRobots && room.otherRobots?.length > 0) ||
      false
    );
  }

  openErrorDialog(data: any) {
    this.dialog.open(ErrorDialogComponent, {
      width: '300px',
      data: data,
    });
  }

  ngOnDestroy(): void {
    this.rosConnectionErrorSubscription?.unsubscribe();
    this.availableSimRoomsSubscription?.unsubscribe();
    this.availableRoomsSubscription?.unsubscribe();
    this.robotSubscription?.unsubscribe();
    this.allSimConnectedSubscription?.unsubscribe();
    
  }
}
