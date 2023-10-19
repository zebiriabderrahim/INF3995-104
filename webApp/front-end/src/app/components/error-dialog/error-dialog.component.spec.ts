import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ErrorDialogComponent } from './error-dialog.component';
import { MAT_DIALOG_DATA, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { of } from 'rxjs';
import { Robot } from 'src/app/interfaces/models';

describe('ErrorDialogComponent', () => {
  let component: ErrorDialogComponent;
  let fixture: ComponentFixture<ErrorDialogComponent>;
  let commandService: CommandService;

  const mockCommandService = {
    getRobots: () => of([]),
    getMissions: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
    terminateSimulation: () => of({}),
  };

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ErrorDialogComponent ],
      imports: [ MatDialogModule ],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: MatDialogRef, useValue: {} },
        { provide: MAT_DIALOG_DATA, useValue: {} }
    ],
    })
    .compileComponents();

    fixture = TestBed.createComponent(ErrorDialogComponent);
    commandService = TestBed.inject(CommandService);
    component = fixture.componentInstance;
    fixture.detectChanges();
    (component as any).dialogRef = { close: () => {} } as MatDialogRef<ErrorDialogComponent>;
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('onCloseClick should terminate mission if the dialog is for the simulation', () => {
    component.data.close = "terminer simulation";
    spyOn(commandService, 'terminateSimulation');
    component.onCloseClick();
    expect(commandService.terminateSimulation).toHaveBeenCalled();
  });

  it('should only close if no data.close is specified', () => {
    component.data.close = null;
    spyOn(commandService, 'terminateSimulation');
    component.onCloseClick();
    expect(commandService.terminateSimulation).not.toHaveBeenCalled();
  });
});
