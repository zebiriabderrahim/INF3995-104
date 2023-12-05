import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MatDialogRef, MAT_DIALOG_DATA, MatDialogModule } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { of } from 'rxjs';
import { ErrorDialogComponent } from './error-dialog.component';
import { SocketService } from 'src/app/services/socket-service/socket.service';

describe('ErrorDialogComponent', () => {
  let component: ErrorDialogComponent;
  let fixture: ComponentFixture<ErrorDialogComponent>;
  let socketService: SocketService;
  let dialogRefSpy: jasmine.SpyObj<MatDialogRef<ErrorDialogComponent>>;
  const mockSocketService = {
    createMissionRoom: () => {},
    viewMissionRoom: () => {},
    terminateSimulation: () => {},
    getAvailableRooms: () => of([]),
    getAvailableMissionRoomsInfo: () => of([]),
    getAvailableSimulatedRoomsInfo: () => of([]),
    navigate: () => {},
    isRoomCreated: of(true),
    isRoomDeleted: of(true),
    isHostLeavingRoom: of(true),
    roomInfo: of({}),
    robots: of([]),
    rosConnectionError: of(false),
    stopBatteryCall: of(false),
    allSimConnected: of(true),
  };

  beforeEach(async () => {
    dialogRefSpy = jasmine.createSpyObj('MatDialogRef', ['close']);

    await TestBed.configureTestingModule({
      declarations: [ ErrorDialogComponent ],
      imports: [ MatDialogModule ],
      providers: [
        { provide: SocketService, useValue: mockSocketService },
        { provide: MatDialogRef, useValue: dialogRefSpy },
        { provide: MAT_DIALOG_DATA, useValue: {} }
      ],
    })
    .compileComponents();

    fixture = TestBed.createComponent(ErrorDialogComponent);
    socketService = TestBed.inject(SocketService);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('onCloseClick should terminate mission and close dialog if the dialog is for the simulation', () => {
    component.data.close = "terminer simulation";
    spyOn(socketService, 'terminateSimulation');
    component.onCloseClick();
    expect(socketService.terminateSimulation).toHaveBeenCalled();
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

    component.password = '  ';
    expect(component.isValidPassword()).toBeFalse();
  });

});
