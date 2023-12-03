import { Component, OnInit, Inject } from '@angular/core';
import { AbstractControl, FormBuilder, FormGroup, Validators} from '@angular/forms';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';

@Component({
  selector: 'app-initial-location',
  templateUrl: './initial-location.component.html',
  styleUrls: ['./initial-location.component.css']
})
export class InitialLocationComponent implements OnInit {
  xValue: number = 0.0;
  yValue: number = 0.0;
  form : FormGroup;

  constructor(public dialogRef: MatDialogRef<InitialLocationComponent>, private formBuilder: FormBuilder, @Inject(MAT_DIALOG_DATA) public data: any) {
    this.form = this.formBuilder.group({
      xValue: ['', [
        Validators.required,
        Validators.pattern(/^-?(0(\.\d{1,2})?|-([0-3](\.\d{1,2})?)|-4(\.0{1,2})?)$/),
      ]],
      yValue: ['', [
        Validators.required,
        Validators.pattern(/^-?(0(\.\d{1,2})?|[0-2](\.\d{1,2})?|-([0-1](\.\d{1,2})?)|-2(\.0{1,2})?)$/),
      ]],
    });
  }

  ngOnInit(): void {}

  defaultPosition() {
    let y = 0.0;

    if (this.data.id == 'robot1') y = 1.0

    const positionData = {
      x: 0.0,
      y: y,
      z: 0.0,
      w: 0.0,
      yaw: 0.0
    };
    this.dialogRef.close(positionData);
  }

  submit(): void { 
    if (this.form.valid) {
      this.xValue = parseFloat(this.xValue.toString()); // -3 to 0
      this.yValue = parseFloat(this.yValue.toString()); // -3 to 3
        
      const positionData = {
        x: this.xValue,
        y: this.yValue,
        z: 0.0,
        w: 0.0,
        yaw: 0.0
      };

      this.dialogRef.close(positionData);
    }
  }

}
