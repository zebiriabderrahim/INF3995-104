import { ComponentFixture, TestBed } from '@angular/core/testing';
import { FormBuilder } from '@angular/forms';
import { MatDialogRef, MAT_DIALOG_DATA } from '@angular/material/dialog';

import { InitialLocationComponent } from './initial-location.component';

describe('InitialLocationComponent', () => {
  let component: InitialLocationComponent;
  let fixture: ComponentFixture<InitialLocationComponent>;
  let dialogRefSpy: jasmine.SpyObj<MatDialogRef<InitialLocationComponent>>;

  beforeEach(async () => {
    dialogRefSpy = jasmine.createSpyObj('MatDialogRef', ['close']);

    await TestBed.configureTestingModule({
      declarations: [InitialLocationComponent],
      providers: [
        { provide: MatDialogRef, useValue: dialogRefSpy },
        { provide: MAT_DIALOG_DATA, useValue: {} },
        FormBuilder
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(InitialLocationComponent);
    component = fixture.componentInstance;

    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize with default x and y values', () => {
    expect(component.xValue).toEqual(0.0);
    expect(component.yValue).toEqual(0.0);
  });

  it('should set default values on calling defaultPosition()', () => {
    let mockData = { id: 'robot1' };
    component.data = mockData;
    component.defaultPosition();

    let expectedPosition = {
      x: -4.0,
      y: 1.0,
      z: 0.0,
      w: 0.0,
      yaw: 0.0
    };

    mockData = { id: 'robot2' };
    component.data = mockData;
    component.defaultPosition();
    expectedPosition.y = 0.0;

    expect(dialogRefSpy.close).toHaveBeenCalledWith(expectedPosition);
  });

  it('should close dialog with correct positionData on calling submit() with valid form', () => {
    spyOnProperty(component.form, 'valid').and.returnValue(true);  
    component.form.value.xValue = 1.0;
    component.form.value.yValue = 2.0;
    component.submit();
  
    const expectedPosition = {
      x: 1.0,
      y: 2.0,
      z: 0.0,
      w: 0.0,
      yaw: 0.0
    };
  
    expect(component.dialogRef.close).toHaveBeenCalledWith(expectedPosition);
  });

  it('should not close dialog on calling submit() with invalid form', () => {
    const mockFormValue = { xValue: '', yValue: '2.0' };
    component.form.setValue(mockFormValue);

    component.submit();

    expect(dialogRefSpy.close).not.toHaveBeenCalled();
  });
});

