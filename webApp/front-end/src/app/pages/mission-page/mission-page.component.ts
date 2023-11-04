import { Component, OnInit, OnDestroy } from '@angular/core';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';
import { CommandService } from 'src/app/services/command-service/command.service';
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
  showLogs = false;
  simulation!: boolean;

  constructor( public socketService: SocketService, private router: Router) {
    this.router = router;
   }

  ngOnInit(): void {

    if (!this.socketService.isConnected()) {
      this.router.navigate(["/home"]);
    }

    this.roomSubscription = this.socketService.getMissionRoomInfo().subscribe((roomInfo: MissionRoom) => {
      if (!this.room) {
        this.room = roomInfo;
        this.socketService.getLogs(this.room.robot);
      }
    });

    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        this.router.navigate(['/home']);
        this.socketService.isHostLeavingRoom.next(false);
        this.socketService.getAvailableRooms();
      }
    });

    this.socketService.simulation.asObservable().subscribe((bool) => {
      this.simulation = bool;
    })
    this.socketService.robots.subscribe((robots: Robot[]) => {
      const robot = robots.find((robot: Robot) => robot.ipAddress === this.room.robot.ipAddress);
      if (robot) {
        this.room.robot.batteryLevel = robot.batteryLevel;
      }
    });
  }

  handleStopMissionClick(robot: Robot): void {
    if (this.simulation) {
      this.socketService.terminateSimulationRobot(robot);
      this.socketService.simulation.next(false);
    }
    else this.socketService.stopMission(robot);
  }

  ngOnDestroy(): void {
    // if (this.roomSubscription) {
    //   this.roomSubscription.unsubscribe();
    // }
    // if (this.isHostLeavingRoomSubscription) {
    //   this.isHostLeavingRoomSubscription.unsubscribe();
    // }
    this.socketService.stopMission(this.room.robot);
  }
}
