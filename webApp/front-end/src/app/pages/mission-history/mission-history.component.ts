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
  missionsCopy: any;
  filteredMissions: any;
  missionsSubscription: Subscription | undefined;
  isHostLeavingRoomSubscription: Subscription | undefined;

  constructor(
    private missionHistoryDialog: MatDialog,
    private commandService: CommandService,
    private socketService: SocketService
  ) {}

  ngOnInit(): void {
    // Subscribe to get the list of missions
    this.missionsSubscription = this.commandService.getMissions().subscribe((data) => {
      this.missions = data;
      this.missionsCopy = [...this.missions];
      this.filteredMissions = this.missions;
    });

    // Subscribe to the event when the host is leaving the room
    this.isHostLeavingRoomSubscription = this.socketService.isHostLeavingRoom.asObservable().subscribe((isHostLeavingRoom: boolean) => {
      if (isHostLeavingRoom) {
        this.socketService.router.navigate(['/home']);
      }
    });
  }

  // Function to filter missions by mode
  showMissionsByMode(mode: string) {
    if (mode === 'Mode simulation') {
      this.filteredMissions = this.missions.filter((mission: any) => mission.type === 'Mode simulation');
    } else if (mode === 'Mode robots physiques') {
      this.filteredMissions = this.missions.filter((mission: any) => mission.type === 'Mode robots physiques');
    } else {
      this.filteredMissions = this.missions;
    }
  }

  // Function to sort missions by distance
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

  // Function to sort missions by duration
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
    // Unsubscribe from all subscriptions to avoid memory leaks
    this.missionsSubscription?.unsubscribe();
    this.isHostLeavingRoomSubscription?.unsubscribe();
  }

  // Function to open the mission overview dialog
  openMissionOverview(mission: any) {
    this.commandService.getMissionMap(mission.name).subscribe((data) => {
      this.missionHistoryDialog.open(MissionHistoryDialog, {
        data: { missionId: mission.name, map: data, duration: mission.duration, type: mission.type, robots: mission.robots, distance: mission.distance },
        width: '80%',
        height: '90%',
      });
    });
  }

  // Function to open the logs overview dialog
  openLogsOverview(mission: any) {
    this.missionHistoryDialog.open(MissionHistoryDialog, {
      data: { missionId: mission.name, logs: mission.logs },
      width: '80%',
      height: '90%',
    });
  }
}
