import { Inject, Component } from '@angular/core';
import { MAT_DIALOG_DATA } from '@angular/material/dialog';

@Component({
  selector: 'mission-history-dialog',
  templateUrl: './mission-history-dialog.component.html',
  styleUrls: ['./mission-history-dialog.component.css']
})
export class MissionHistoryDialog {

  constructor(@Inject(MAT_DIALOG_DATA) public data: { missionid: number }) {}
  
}
