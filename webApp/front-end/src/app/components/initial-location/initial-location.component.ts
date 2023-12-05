import { Component, OnInit, Inject } from '@angular/core';
import { FormBuilder, FormGroup, Validators } from '@angular/forms';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';

@Component({
  selector: 'app-initial-location',
  templateUrl: './initial-location.component.html',
  styleUrls: ['./initial-location.component.css']
})
export class InitialLocationComponent implements OnInit {
  xValue: number = 0.0;
  yValue: number = 0.0;
  form: FormGroup;

  // Constructor to initialize the form with validation rules
  constructor(
    public dialogRef: MatDialogRef<InitialLocationComponent>,
    private formBuilder: FormBuilder,
    @Inject(MAT_DIALOG_DATA) public data: any
  ) {
    // Form initialization with validation rules for xValue and yValue
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

  // Set default position based on robot ID
  defaultPosition() {
    let y = (this.data.id == 'robot1') ? 1.0 : 0.0;
    let x = -4.0;

    const positionData = {
      x: x,
      y: y,
      z: 0.0,
      w: 0.0,
      yaw: 0.0
    };
    this.dialogRef.close(positionData);
  }

  // Submit form data if valid
  submit(): void {
    if (this.form.valid) {
      // Parse form values and close the dialog with position data
      this.xValue = parseFloat(this.form.value.xValue.toString());
      this.yValue = parseFloat(this.form.value.yValue.toString());

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
