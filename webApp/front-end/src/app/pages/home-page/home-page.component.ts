import { Component, OnInit } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';
import { Robot } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';

@Component({
  selector: 'app-home-page',
  templateUrl: './home-page.component.html',
  styleUrls: ['./home-page.component.css'],
})
export class HomePageComponent implements OnInit {
  title = 'front-end';
  simulationStatus: boolean = false;
  robots: Robot[] = [
    {
      name: 'Robot 1',
      ipAddress: '192.168.0.110',
      state: 'On',
      batteryLevel: 100,
    },
    {
      name: 'Robot 2',
      ipAddress: '192.168.0.122',
      state: 'On',
      batteryLevel: 100,
    },
  ];

  constructor(
    private commandService: CommandService,
    private dialog: MatDialog
  ) {}

  ngOnInit(): void {
    if (this.robots.length === 0) {
      this.openErrorDialog({
        title: 'Error',
        message: 'No robots found.',
        close: 'close',
      });
    }
  }

  handleSimulation() {
    this.commandService.simulateMission();
    this.openErrorDialog({
      title: 'Avertissement',
      message: 'Veuillez regarder la simulation sur Gazebo',
      close: 'terminer simulation',
    });
  }

  openErrorDialog(data: any) {
    this.dialog.open(ErrorDialogComponent, {
      width: '300px',
      data: data,
    });
  }
}