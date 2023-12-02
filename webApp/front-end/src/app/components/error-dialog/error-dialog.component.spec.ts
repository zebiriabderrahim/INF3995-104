import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MatDialogRef, MAT_DIALOG_DATA, MatDialogModule } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { of } from 'rxjs';
import { ErrorDialogComponent } from './error-dialog.component';

describe('ErrorDialogComponent', () => {
  let component: ErrorDialogComponent;
  let fixture: ComponentFixture<ErrorDialogComponent>;
  let commandService: CommandService;
  let dialogRefSpy: jasmine.SpyObj<MatDialogRef<ErrorDialogComponent>>;

  const mockCommandService = {
    getRobots: () => of([]),
    getMissions: () => of([]),
    identifyRobot: () => of({}),
    simulateMission: () => of({}),
    terminateSimulation: () => of({}),
  };

  beforeEach(async () => {
    dialogRefSpy = jasmine.createSpyObj('MatDialogRef', ['close']);

    await TestBed.configureTestingModule({
      declarations: [ ErrorDialogComponent ],
      imports: [ MatDialogModule ],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: MatDialogRef, useValue: dialogRefSpy },
        { provide: MAT_DIALOG_DATA, useValue: {} }
      ],
    })
    .compileComponents();

    fixture = TestBed.createComponent(ErrorDialogComponent);
    commandService = TestBed.inject(CommandService);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('onCloseClick should terminate mission and close dialog if the dialog is for the simulation', () => {
    component.data.close = "terminer simulation";
    spyOn(commandService, 'terminateSimulation');
    component.onCloseClick();
    expect(commandService.terminateSimulation).toHaveBeenCalled();
    expect(dialogRefSpy.close).toHaveBeenCalled();
  });

  it('onCloseClick should close dialog with the correct argument for quitter', () => {
    component.data.close = "quitter";
    component.onCloseClick();
    expect(dialogRefSpy.close).toHaveBeenCalledWith(component.data.close);
  });

  it('should only close dialog if no data.close is specified', () => {
    component.data.close = null;
    component.onCloseClick();
    expect(dialogRefSpy.close).toHaveBeenCalled();
  });

  it('onCloseClick should close dialog with password for valid editer robots action', () => {
    component.data.close = "editer robots";
    component.password = 'validPassword';
    spyOn(component, 'isValidPassword').and.returnValue(true);
    component.onCloseClick();
    expect(dialogRefSpy.close).toHaveBeenCalledWith(component.password);
  });

  it('onSaveClick should close dialog with the correct argument for sauvegarder', () => {
    component.data.save = "sauvegarder";
    component.onSaveClick();
    expect(dialogRefSpy.close).toHaveBeenCalledWith(component.data.save);
  });

  it('isValidPassword should return true for a valid password', () => {
    component.password = 'validPassword';
    expect(component.isValidPassword()).toBeTrue();
  });  

  it('isValidPassword should return false for an invalid password', () => {
    component.password = '';
    expect(component.isValidPassword()).toBeFalse();
  
    component.password = '  '; // Password with only spaces
    expect(component.isValidPassword()).toBeFalse();
  });
   
});
