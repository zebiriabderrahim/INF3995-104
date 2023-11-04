import { ComponentFixture, TestBed, fakeAsync, flush, tick } from '@angular/core/testing';

import { HomePageComponent } from './home-page.component';
import { MAT_DIALOG_DATA, MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';
import { CommandService } from 'src/app/services/command-service/command.service';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { LogBoxComponent } from 'src/app/components/log-box/log-box.component';
import { RobotComponent } from 'src/app/components/robot/robot.component';
import { NgModule } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { BehaviorSubject, Subject, of } from 'rxjs';
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
  const getTestRobot =(): Robot => ({
    name: "test",
    ipAddress: "0.0.0.0",
    state: "on",
    batteryLevel: 100
  });
  

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
    getAvailableSimulatedRoomsInfo: () => of([]),
    navigate: (path: string) => {},
    isRoomCreated: new Subject<boolean>(),
    isRoomDeleted: new Subject<boolean>(),
    isHostLeavingRoom: new Subject<boolean>(),
    roomInfo: new Subject<MissionRoom>(),
    robots: new BehaviorSubject<Robot[]>([getTestRobot()]),
    rosConnectionError: new Subject<boolean>(),
    stopBatteryCall: new BehaviorSubject<Boolean>(false),
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

  // afterEach(() => {
  //   component.ngOnDestroy();
  // });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  // it('ngOnInit should init the robots properly', () => {
  //   component.robots = [];
  //   component.ngOnInit();
  //   mockSocketService.robots.next([getTestRobot()]);
  //   expect(component.robots).toEqual([getTestRobot()]);
  // });

  it('ngOnInit should open an Error Dialog if ros connection failed', fakeAsync(() => {
    component.robots = [getTestRobot()];
    const dialogSpy = spyOn((component as any).dialog, 'open').and.stub();
    
    component.ngOnInit();
    tick(); // Tick some time to ensure ngOnInit asynchronous operations are completed.
    
    expect(dialogSpy).not.toHaveBeenCalled();
    
    mockSocketService.rosConnectionError.next(true);
    tick(); // Tick again to ensure the error handling asynchronous operations are completed.
  
    expect(dialogSpy).toHaveBeenCalledWith(ErrorDialogComponent, { width: '300px', data: {
      title: 'Error',
      message: 'Ros connection not established',
      close: 'close',
    }});
  
    // flush(); // This ensures that there are no more remaining asynchronous activities.
  }));
  

  it('handleSimulation should call simulateMission', () => {
    spyOn(commandService, 'simulateMission');
    component.handleSimulation();
    expect(commandService.simulateMission).toHaveBeenCalled();
  });

  it('openErrorDialog should open errorDialog', () => {
    const dialogSpy = spyOn((component as any).dialog, 'open').and.stub();
    component.openErrorDialog('test');
    expect(dialogSpy).toHaveBeenCalledWith(ErrorDialogComponent, { width: '300px', data: 'test' });
  });
});