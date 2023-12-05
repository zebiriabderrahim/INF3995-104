import { Component, Inject, OnInit } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { FormsModule } from '@angular/forms';
import { SocketService } from 'src/app/services/socket-service/socket.service';


@Component({
  selector: 'app-error-dialog',
  templateUrl: './error-dialog.component.html',
  styleUrls: ['./error-dialog.component.css']
})
export class ErrorDialogComponent implements OnInit {
  password: string = ''; // Variable to store the password

  constructor(
    public dialogRef: MatDialogRef<ErrorDialogComponent>,
    @Inject(MAT_DIALOG_DATA) public data: any,
    private socketService: SocketService
  ) {}

  ngOnInit(): void {}

  onCloseClick(): void {
    // Check the value of 'close' in data and perform actions accordingly
    if (this.data.close === "terminer simulation") {
      this.socketService.terminateSimulation();
      this.dialogRef.close();
    } else if (this.data.close === 'quitter') {
      this.dialogRef.close(this.data.close);
    } else if (this.data.close == 'editer robots' && this.isValidPassword()) {
      this.dialogRef.close(this.password);
    } else {
      this.dialogRef.close();
    }
  }

  onSaveClick(): void {
    // Check the value of 'save' in data and close the dialog accordingly
    if (this.data.save === 'sauvegarder') {
      this.dialogRef.close(this.data.save);
    }
  }

  isValidPassword(): boolean {
    // Check if the password is valid (non-empty)
    return this.password.trim().length > 0;
  }
}
