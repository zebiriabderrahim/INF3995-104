import { AfterViewInit, Component, ElementRef, Inject, NgModule, OnInit, ViewChild } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { FormsModule } from '@angular/forms';


@Component({
  selector: 'app-error-dialog',
  templateUrl: './error-dialog.component.html',
  styleUrls: ['./error-dialog.component.css']
})
export class ErrorDialogComponent implements OnInit {
  password: string = '';

  constructor(public dialogRef: MatDialogRef<ErrorDialogComponent>, @Inject(MAT_DIALOG_DATA) public data: any, private commandService: CommandService) { }

  ngOnInit(): void {}

  onCloseClick(): void {
    if (this.data.close === "terminer simulation") {
      this.commandService.terminateSimulation();
      this.dialogRef.close();
    }
    else if (this.data.close === "quitter") {
        this.dialogRef.close(this.data.close);
    }
    else if (this.data.close == "editer robots" && this.isValidPassword()) {
      this.dialogRef.close(this.password);
    } else {
      this.dialogRef.close();
    }
  }

  onSaveClick(): void {
    if (this.data.save === "sauvegarder") {
      this.dialogRef.close(this.data.save);
    }
  }

  isValidPassword(): boolean {
    return this.password.trim().length > 0;
  }
}
