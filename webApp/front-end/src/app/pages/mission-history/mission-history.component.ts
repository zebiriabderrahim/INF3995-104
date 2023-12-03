import { Component, OnInit, OnDestroy } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { MissionHistoryDialog } from 'src/app/components/mission-history-dialog/mission-history-dialog.component';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';

@Component({
  selector: 'app-mission-history',
  templateUrl: './mission-history.component.html',
  styleUrls: ['./mission-history.component.css']
})
export class MissionHistoryComponent implements OnInit, OnDestroy {
  missions: any;
  missionsSubscription: Subscription | undefined;
  isHostLeavingRoomSubscription: Subscription | undefined;

  constructor(private missionHistoryDialog: MatDialog, private commandService: CommandService, private socketService: SocketService) { }

  ngOnInit(): void {
    this.missionsSubscription = this.commandService.getMissions().subscribe((data) => {
            this.missions = data;
    });

    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        this.socketService.router.navigate(['/home']);
      }
    });
  }

  ngOnDestroy(): void {
      this.missionsSubscription?.unsubscribe();
      this.isHostLeavingRoomSubscription?.unsubscribe();
  }

  openMissionOverview(mission: any) {
    this.commandService.getMissionMap(mission.name).subscribe((data) => {
      this.missionHistoryDialog.open(MissionHistoryDialog, {
        data: { missionid: mission.name, map: data, duration: mission.duration, type: mission.type, robots: mission.robots, distance: mission.distance },
        width: '80%',
        height: '90%',
      });
    });
  }

  openLogsOverview(mission: any) {
    this.missionHistoryDialog.open(MissionHistoryDialog, {
      data: { missionid: mission.name, logs: mission.logs },
      width: '80%',
      height: '90%',
  });
  }
}
