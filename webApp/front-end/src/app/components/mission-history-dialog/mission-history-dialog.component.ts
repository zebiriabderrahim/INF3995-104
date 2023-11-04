import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Inject, Component } from '@angular/core';
import { MAT_DIALOG_DATA } from '@angular/material/dialog';
import { Log } from 'src/app/interfaces/models';
import { BrowserModule } from '@angular/platform-browser'

@Component({
  selector: 'mission-history-dialog',
  templateUrl: './mission-history-dialog.component.html',
  styleUrls: ['./mission-history-dialog.component.css'],
})
export class MissionHistoryDialog {
  logs: Log[];
  logShown: boolean = false;

  constructor(@Inject(MAT_DIALOG_DATA) public data: { missionid: number, logs: Log[] }) {
    if (data.logs){
      this.logShown = true;
      this.logs = data.logs;
    }
    else this.logs = [];
  }
  
}
