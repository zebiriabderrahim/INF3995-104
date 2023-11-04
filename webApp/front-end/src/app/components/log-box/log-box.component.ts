import { AfterViewChecked, Component, ElementRef, Input, ViewChild } from '@angular/core';
import { Log } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';

@Component({
  selector: 'app-log-box',
  templateUrl: './log-box.component.html',
  styleUrls: ['./log-box.component.css'],
})

export class LogBoxComponent implements AfterViewChecked {
  @ViewChild('logboxElement', { static: false }) logboxElement!: ElementRef;
  logs: Log[] = [];
  logsShown: Log[] = [];
  systemChecked = true;
  commandChecked = true;
  otherChecked = true;

  constructor(private socketService: SocketService) { 

    this.socketService.currentLogs.subscribe((logs) => {
      this.logs = logs;
      this.filterLogs();
    });
  }

  filterLogs() {
    this.logsShown = this.logs.filter(log => {
      if (!this.systemChecked && log.type === 'system') return false;
      if (!this.commandChecked && log.type === 'command') return false;
      if (!this.otherChecked && log.type === 'other') return false;
      return true;
    });
  }

  ngAfterViewChecked() {
    this.scrollToBottom();
  }

  scrollToBottom(): void {
    try {
      this.logboxElement.nativeElement.scrollTop = this.logboxElement.nativeElement.scrollHeight;
    } catch (err) { }
  }

}
