import { Component, OnInit, OnDestroy} from '@angular/core';
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
  missionsCopy: any;
  filteredMissions: any;
  missionsSubscription: Subscription | undefined;
  isHostLeavingRoomSubscription: Subscription | undefined;

  constructor(private missionHistoryDialog: MatDialog, private commandService: CommandService, private socketService: SocketService) {}

  ngOnInit(): void {
    this.missionsSubscription = this.commandService.getMissions().subscribe((data) => {
      this.missions = data;
      this.missionsCopy = [...this.missions]
      this.filteredMissions = this.missions
    });

    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        this.socketService.router.navigate(['/home']);
      }
    });
  }

  showMissionsByMode(mode: string) {
    if (mode === 'Mode simulation') {
      this.filteredMissions = this.missions.filter((mission: any) => mission.type === 'Mode simulation');
    } else if (mode === 'Mode robots physiques') {
      this.filteredMissions = this.missions.filter((mission: any) => mission.type === 'Mode robots physiques');
    } else {
      this.filteredMissions = this.missions;
    }
  }

  sortMissionsByDistance() {
      this.filteredMissions = this.missions.sort((a: any, b: any) => {
        const sumDistances = (str: string) => {
          const matches = str.match(/\d+\.\d+/g) || [];
          return matches.reduce((acc, val) => acc + parseFloat(val), 0);
        };

        const distanceA = sumDistances(a.distance);
        const distanceB = sumDistances(b.distance);

        return distanceA - distanceB;
      });
  }
  
  
  sortMissionsByDuration() {
      this.filteredMissions = this.missions.sort((a: any, b: any) => {
        const extractDuration = (str: string) => {
          const matches = str.match(/\d+/g) || [];
          const [minutes, seconds] = matches.map(Number);
          return minutes * 60 + seconds;
        };
  
        const durationA = extractDuration(a.duration);
        const durationB = extractDuration(b.duration);
  
        return durationA - durationB;
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
