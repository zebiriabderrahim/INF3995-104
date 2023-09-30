import { Component, OnInit, OnDestroy } from '@angular/core';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';

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

  constructor(private socketService: SocketService) { }

  ngOnInit(): void {
    if (!this.socketService.isConnected()) {
      this.socketService.router.navigate(["/home"]);
    }

    this.roomSubscription = this.socketService.getMissionRoomInfo().subscribe((roomInfo: MissionRoom) => {
      if (!this.room) {
        this.room = roomInfo;
      }
    });

    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        this.socketService.router.navigate(['/home']);
        this.socketService.isHostLeavingRoom.next(false);
        this.socketService.getAvailableRooms();
      }
    });
  }

  handleStopMissionClick(robot: Robot): void {
    this.socketService.stopMission(robot);
  }

  ngOnDestroy(): void {
    if (this.roomSubscription) {
      this.roomSubscription.unsubscribe();
    }
    if (this.isHostLeavingRoomSubscription) {
      this.isHostLeavingRoomSubscription.unsubscribe();
    }
  }
}
