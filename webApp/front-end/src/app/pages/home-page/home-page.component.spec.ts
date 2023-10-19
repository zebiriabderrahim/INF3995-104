import { ComponentFixture, TestBed } from '@angular/core/testing';

import { HomePageComponent } from './home-page.component';
import { MAT_DIALOG_DATA, MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';
import { CommandService } from 'src/app/services/command-service/command.service';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { LogBoxComponent } from 'src/app/components/log-box/log-box.component';
import { RobotComponent } from 'src/app/components/robot/robot.component';
import { NgModule } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { Subject, of } from 'rxjs';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

@NgModule({
  imports: [FormsModule, BrowserAnimationsModule],
})
export class DynamicTestModule {}

describe('HomePageComponent', () => {
  let component: HomePageComponent;
  let fixture: ComponentFixture<HomePageComponent>;
  let commandService: CommandService;

  const mockCommandService = {
    getRobots: () => of([]),
    getMissions: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
    terminateSimulation: () => of({}),
  };

  const mockSocketService = {
    createMissionRoom: (robot: Robot) => {},
    viewMissionRoom: (robot: Robot) => {},
    getAvailableRooms: () => of([]),
    getAvailableMissionRoomsInfo: () => of([]),
    isRoomCreated: new Subject<boolean>(),
    isRoomDeleted: new Subject<boolean>(),
    isHostLeavingRoom: new Subject<boolean>(),
    roomInfo: new Subject<MissionRoom>(),
  };

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ HomePageComponent, LogBoxComponent, ErrorDialogComponent, RobotComponent ],
      imports: [DynamicTestModule, MatDialogModule],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: MAT_DIALOG_DATA, useValue: {} },
        { provide: SocketService, useValue: mockSocketService },
    ],
    })
    .compileComponents();

    fixture = TestBed.createComponent(HomePageComponent);
    commandService = TestBed.inject(CommandService);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('ngOnInit should open an Error Dialog if robots is empty', () => {
    component.robots = [];
    const dialogSpy = spyOn((component as any).dialog, 'open').and.stub();
    component.ngOnInit();
    expect(dialogSpy).toHaveBeenCalledWith(ErrorDialogComponent, { width: '300px', data: {
      title: 'Error',
      message: 'No robots found.',
      close: 'close',
    }});
  });

  it('ngOnInit should not open an Error Dialog if robots is not empty', () => {
    component.robots = [getTestRobot()];
    const dialogSpy = spyOn((component as any).dialog, 'open').and.stub();
    component.ngOnInit();
    expect(dialogSpy).not.toHaveBeenCalled();
  });

  it('handleSimulation should call simulateMission', () => {
    spyOn(commandService, 'simulateMission');
    spyOn(component, 'openErrorDialog');
    component.handleSimulation();
    expect(commandService.simulateMission).toHaveBeenCalled();
    expect(component.openErrorDialog).toHaveBeenCalledWith({
      title: 'Avertissement',
      message: 'Veuillez regarder la simulation sur Gazebo',
      close: 'terminer simulation',
    });
  });

  it('openErrorDialog should open errorDialog', () => {
    const dialogSpy = spyOn((component as any).dialog, 'open').and.stub();
    component.openErrorDialog('test');
    expect(dialogSpy).toHaveBeenCalledWith(ErrorDialogComponent, { width: '300px', data: 'test' });
  });
});

const getTestRobot =(): Robot => ({
  name: "test",
  ipAddress: "0.0.0.0",
  state: "on",
  batteryLevel: 100
});
