import { Component } from '@angular/core';
import { Log } from 'src/app/interfaces/models';

@Component({
  selector: 'app-log-box',
  templateUrl: './log-box.component.html',
  styleUrls: ['./log-box.component.css'],
})

export class LogBoxComponent {
  log = '';
  logs: Log[] = [{
    type: 'command',
    name: 'command',
    message: 'le robot retourne a la base',
    timestamp: 'sep 13 12:00:01'
  },
  {
    type: 'system',
    name: 'system',
    message: 'la batterie est faible',
    timestamp: 'sep 13 13:02:05'
  },
  {
    type: 'other',
    name: 'lidar',
    message: 'objet detecte a 1.3m',
    timestamp: 'sep 13 13:04:06'
  }];
  logsShown: Log[] = this.logs;


  systemChecked = true;
  commandChecked = true;
  otherChecked = true;

  filterLogs(logType: String, wasChecked: boolean) {
    if (wasChecked) {
      this.logsShown = this.logsShown.filter(log => log.type !== logType);
    } else {
      this.logsShown = this.logsShown.concat(this.logs.filter(log => log.type === logType));
    }
  }

}
