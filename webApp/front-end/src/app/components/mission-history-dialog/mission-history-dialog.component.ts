import { Inject, Component } from '@angular/core';
import { MAT_DIALOG_DATA } from '@angular/material/dialog';
import { Log } from 'src/app/interfaces/models';

@Component({
  selector: 'mission-history-dialog',
  templateUrl: './mission-history-dialog.component.html',
  styleUrls: ['./mission-history-dialog.component.css'],
})
export class MissionHistoryDialog {
  logs: Log[];
  map: number[];
  duration!: string;
  type!: string;
  robots!: string;
  distance!: string;
  logShown: boolean = false;

  // Constructor with dependency injection for MAT_DIALOG_DATA
  constructor(@Inject(MAT_DIALOG_DATA) public data: { missionId: number, logs: Log[], map: number[], duration: string, type: string, robots: string, distance: string}) {
    // Check if logs are provided in the data
    if (data.logs){
      this.logShown = true;
      this.logs = data.logs;
      this.map = [];
    }
    else {
      // If logs are not provided, set logShown to false and initialize other properties
      this.logs = [];
      this.logShown = false;
      this.map = data.map;
      this.duration = data.duration;
      this.type = data.type;
      this.robots = data.robots;
      this.distance = data.distance;
    }
  }
}
