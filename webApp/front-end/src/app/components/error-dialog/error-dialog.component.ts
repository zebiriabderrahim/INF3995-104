import { Component, Inject, OnInit } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';

@Component({
  selector: 'app-error-dialog',
  templateUrl: './error-dialog.component.html',
  styleUrls: ['./error-dialog.component.css']
})
export class ErrorDialogComponent implements OnInit {

  constructor(public dialogRef: MatDialogRef<ErrorDialogComponent>, @Inject(MAT_DIALOG_DATA) public data: any, private commandService: CommandService) { }

  ngOnInit(): void {}

  onCloseClick(): void {
    if (this.data.close == "terminer simulation")
      this.commandService.terminateSimulation();
    this.dialogRef.close();
  }
}
