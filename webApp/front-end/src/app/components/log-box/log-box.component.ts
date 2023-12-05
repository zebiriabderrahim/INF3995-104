import { AfterViewChecked, Component, ElementRef, ViewChild } from '@angular/core';
import { Subscription } from 'rxjs';
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
  currentLogsSubscription: Subscription | undefined;

  constructor(private socketService: SocketService) {
    // Subscribe to the currentLogs observable to receive log updates
    this.currentLogsSubscription = this.socketService.currentLogs.subscribe((logs) => {
      this.logs = logs;
      this.filterLogs(); // Call filterLogs to update the displayed logs based on filter settings
    });
  }

  // Filter logs based on the selected log types (system, command, other)
  filterLogs() {
    this.logsShown = this.logs.filter((log) => {
      if (!this.systemChecked && log.type === 'system') return false;
      if (!this.commandChecked && log.type === 'command') return false;
      if (!this.otherChecked && log.type === 'other') return false;
      return true;
    });
  }

  // Delayed filterLogs to be used in situations where the view is not updated immediately
  delayedFilterLogs() {
    setTimeout(() => {
      this.filterLogs();
    }, 0);
  }

  // After the view is checked, scroll to the bottom of the logbox
  ngAfterViewChecked() {
    this.scrollToBottom();
  }

  // Scroll to the bottom of the logbox
  scrollToBottom(): void {
    try {
      this.logboxElement.nativeElement.scrollTop = this.logboxElement.nativeElement.scrollHeight;
    } catch (err) {}
  }

  // Unsubscribe from the currentLogs observable to avoid memory leaks
  ngOnDestroy(): void {
    this.currentLogsSubscription?.unsubscribe();
  }
}
