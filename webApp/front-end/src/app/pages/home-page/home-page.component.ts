import { Component, OnInit } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { Subscription } from 'rxjs';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';

@Component({
  selector: 'app-home-page',
  templateUrl: './home-page.component.html',
  styleUrls: ['./home-page.component.css'],
})
export class HomePageComponent implements OnInit {
  title = 'front-end';
  simulationStatus: boolean = false;
  simulationRooms: MissionRoom[];
  availableSimRoomsSubscription: Subscription | undefined;
  robots: Robot[] = [];
  rosConnectionErrorSubscription: Subscription | undefined;

  constructor(
    private commandService: CommandService,
    private dialog: MatDialog,
    private socketService: SocketService
  ) {
    this.simulationRooms = [];
  }

  ngOnInit(): void {
    this.socketService.robots.subscribe((robots: Robot[]) => {
      this.robots = robots;
    });

    this.rosConnectionErrorSubscription=this.socketService.rosConnectionError.subscribe(rosConnectionError => {
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
  }

  handleSimulation() {
    this.commandService.simulateMission();
  }

  openErrorDialog(data: any) {
    this.dialog.open(ErrorDialogComponent, {
      width: '300px',
      data: data,
    });
  }

  ngOnDestroy(): void {
    this.rosConnectionErrorSubscription?.unsubscribe();
    // if (this.availableSimRoomsSubscription) {
    //   this.availableSimRoomsSubscription.unsubscribe();
    // }
    // if (this.socketService.rosConnectionError) {
    //   this.socketService.rosConnectionError.unsubscribe();
    // }
    // if (this.socketService.robots) {
    //   this.socketService.robots.unsubscribe();
    // }
  }
}
