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
    // Fetch available rooms and subscribe to relevant socket service events
    this.socketService.getAvailableRooms();

    // Subscribe to the event indicating the status of all simulated connections
    this.allSimConnectedSubscription = this.socketService.allSimConnected.subscribe((allSimConnected: boolean) => {
      this.enableSimulation = allSimConnected;
    });

    // Subscribe to the list of robots
    this.robotSubscription = this.socketService.robots.subscribe((robots: Robot[]) => {
      this.robots = robots;
      // Check if both robots are in 'On' state
      if (robots[0].state === 'On' && robots[1].state === 'On') {
        this.areRobotsOn = true;
      }
    });

    // Subscribe to ROS connection errors
    this.rosConnectionErrorSubscription = this.socketService.rosConnectionError.subscribe(rosConnectionError => {
      // If there is a ROS connection error, open an error dialog
      if (rosConnectionError) {
        this.openErrorDialog({
          title: 'Error',
          message: 'ROS connection not established',
          close: 'close',
        });
        this.socketService.rosConnectionError.next(false);
      }

      // Navigate to the home page
      this.socketService.navigate('/home');
    });

    // Subscribe to available simulated rooms
    this.availableSimRoomsSubscription = this.socketService.getAvailableSimulatedRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.simulationRooms = rooms;
    });

    // Subscribe to available mission rooms
    this.availableRoomsSubscription = this.socketService.getAvailableMissionRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.availableRooms = rooms;
    });
  }

  // Function to handle simulation
  handleSimulation() {
    this.commandService.simulateMission();
  }

  // Function to handle launching all robots
  handleRobots() {
    this.commandService.launchAllRobots(this.robots);
  }

  // Function to view mission details
  viewMission() {
    this.commandService.viewMission();
  }

  // Function to check if both robots are used in a set of rooms
  bothRobotsUsed(rooms: MissionRoom[]) {
    return (
      rooms.some(room => room.otherRobots && room.otherRobots?.length > 0) ||
      false
    );
  }

  // Function to open an error dialog
  openErrorDialog(data: any) {
    this.dialog.open(ErrorDialogComponent, {
      width: '300px',
      data: data,
    });
  }

  ngOnDestroy(): void {
    // Unsubscribe from all subscriptions to avoid memory leaks
    this.rosConnectionErrorSubscription?.unsubscribe();
    this.availableSimRoomsSubscription?.unsubscribe();
    this.availableRoomsSubscription?.unsubscribe();
    this.robotSubscription?.unsubscribe();
    this.allSimConnectedSubscription?.unsubscribe();
  }
}
