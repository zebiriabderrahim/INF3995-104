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
  let socketService:SocketService
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
    launchAllRobots:(robots: Robot[]) => of({}),
    viewMission: () => of({}),
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
    rosConnectionError: new BehaviorSubject<boolean>(false),
    stopBatteryCall: new BehaviorSubject<Boolean>(false),
    allSimConnected: new Subject<boolean>(),


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
    component = fixture.componentInstance;
    commandService = TestBed.inject(CommandService);
    socketService = TestBed.inject(SocketService);
  });

  // afterEach(() => {
  //   component.ngOnDestroy();
  // });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('ngOnInit should init the robots properly', () => {
    component.robots = [];
    component.ngOnInit();
    mockSocketService.robots.next([getTestRobot()]);
    expect(component.robots).toEqual([getTestRobot()]);
  });

  it('ngOnInit should open an Error Dialog if ros connection failed', fakeAsync(() => {
    component.robots = [getTestRobot()];
    const dialogSpy = spyOn((component as any).dialog, 'open').and.stub();
  
    component.ngOnInit();
    tick();
  
    expect(dialogSpy).not.toHaveBeenCalled();
  
    mockSocketService.rosConnectionError.next(true);
    tick();
  
    expect(dialogSpy).toHaveBeenCalledWith(ErrorDialogComponent, jasmine.any(Object));
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

  it('should update enableSimulation when allSimConnected emits', () => {
    const allSimConnectedValue = true;
    spyOn(socketService.allSimConnected, 'subscribe').and.callThrough();
    component.ngOnInit();
    socketService.allSimConnected.next(allSimConnectedValue);
    expect(component.enableSimulation).toBe(allSimConnectedValue);
  });

  it('should return true if any room has other robots', () => {
    const getTestRobot =(): Robot => ({
      name: "test",
      ipAddress: "0.0.0.0",
      state: "on",
      batteryLevel: 100
    });
    
    const getTestMission = (): MissionRoom => ({
      hostId: "0.0.0.0",
      robot: getTestRobot(),
      guestId: [],
      otherRobots: [getTestRobot()]
    });
    const roomsWithOtherRobots: MissionRoom[] = [getTestMission()];
    expect(component.bothRobotsUsed(roomsWithOtherRobots)).toBeTrue();
  });
  
  it('should return true if any room has other robots', () => {
    const getTestRobot =(): Robot => ({
      name: "test",
      ipAddress: "0.0.0.0",
      state: "on",
      batteryLevel: 100
    });
    
    const getTestMission = (): MissionRoom => ({
      hostId: "0.0.0.0",
      robot: getTestRobot(),
      guestId: [],
    });
    const roomsWithOtherRobots: MissionRoom[] = [getTestMission()];
    expect(component.bothRobotsUsed(roomsWithOtherRobots)).toBeFalsy();
  });
  

  it('should update robots and areRobotsOn when robots emits', () => {
    const testRobots: Robot[] = [ {name: 'robot1', ipAddress: '192..168.0.1', state: 'On', batteryLevel: 100},
                                  {name: 'robot2', ipAddress: '192..168.0.2', state: 'On', batteryLevel: 100}
  ];
    spyOn(socketService.robots, 'subscribe').and.callThrough();
    component.ngOnInit();
    socketService.robots.next(testRobots);
    expect(component.robots).toEqual(testRobots);
    expect(component.areRobotsOn).toBeTrue();
  });

  it('should call launchAllRobots on handleRobots', () => {
    spyOn(commandService, 'launchAllRobots').and.stub();
    component.robots = [getTestRobot(), getTestRobot()];
    component.handleRobots();
    expect(commandService.launchAllRobots).toHaveBeenCalledWith(component.robots);
  });
  it('should call viewMission on viewMission', () => {
    spyOn(commandService, 'viewMission').and.stub();
    component.viewMission();
    expect(commandService.viewMission).toHaveBeenCalled();
  });
    
  
});