import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { RobotComponent } from './robot.component';
import { MissionRoom, Robot, RobotBatteryInfo } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { BehaviorSubject, Observable, Subject, of } from 'rxjs';
import { NgModule, SimpleChange } from '@angular/core';
import { MatDialog,MatDialogModule } from '@angular/material/dialog';


@NgModule({
  imports: [MatDialogModule],
})
export class DynamicTestModule {}
describe('RobotComponent', () => {
  let component: RobotComponent;
  let fixture: ComponentFixture<RobotComponent>;
  let socketService: SocketService;
  let commandService: CommandService;
  let dialog: MatDialog;


  const mockCommandService = {
    getMissions: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
    createMissionRoom: (robot: Robot) => of({}),
    viewMissionRoom: (robot: Robot) => of({}),
    simulateMissionRobot: (robot: Robot) => of({}),
  };

  const mockSocketService = {
    stopMission: () => {},
    asObservable: () => {},
    returnToBaseSimulation:(robot:Robot) => {},
    returnToBase:(robot:Robot)=> {},
    terminateSimulationRobot: () => {},
    terminateAllPhysicalRobots: (robot:Robot) => {},
    isConnected: (bool:boolean) => {return bool;},
    getAvailableRooms:()=> {},
    getLogs:(room:MissionRoom,bool:boolean )=>{},
    setInitialPosition: (robotName: string, location: string) => of({}),

    getBatteryLevel:(robot:Robot)=>{},
    getBatteryLevelSim:(robot:Robot)=>{},
    getAvailableMissionRoomsInfo : () => {
      return mockSocketService.availableRooms.asObservable();
    },

    allPhysicalRobots: new BehaviorSubject<boolean>(false),
    availableRooms: new BehaviorSubject<MissionRoom[]>([]),
    roomInfo: new Subject<MissionRoom[]>(),
    simulation: new BehaviorSubject<boolean>(false),
    batteryLevelSimulation: new Subject<RobotBatteryInfo>(),
    robots: new Subject<Robot[]>(),
    isRoomDeleted: new Subject<boolean>(),
    isHostLeavingRoom: new Subject<boolean>(),
    stopBatteryCallSimulation: new Subject<boolean>(),
    isRoomCreated: new Subject<boolean>(),
    stopBatteryCall: new Subject<boolean>(),
    simulationRooms: new Subject<MissionRoom[]>(),
    getAvailableSimulatedRoomsInfo: () => {
      return mockSocketService.simulationRooms.asObservable();
    },
    getMissionRoomInfo: ()=> {
      return mockSocketService.roomInfo.asObservable();
    }

   };
  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ RobotComponent ],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: SocketService, useValue: mockSocketService },
        MatDialog,
    ],
    imports: [
      MatDialogModule,
    ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotComponent);
    component = fixture.componentInstance;
    socketService = TestBed.inject(SocketService);
    commandService = TestBed.inject(CommandService);
    dialog = TestBed.inject(MatDialog);
    component.robot = getTestRobot();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });


  it('launchMission should call createMissionClick', () => {
    const createMissionSpy = spyOn(commandService, 'createMissionRoom').and.returnValue();
    component.launchMission(getTestRobot());
    expect(createMissionSpy).toHaveBeenCalled();
  });

  it('indentifyRobot should call identifyRobot', () => {
    const identifyRobotSpy = spyOn(commandService, 'identifyRobot').and.returnValue();
    component.identifyRobot(getTestRobot());
    expect(identifyRobotSpy).toHaveBeenCalled();
  });

  it('viewMission should call viewMissionRoom', () => {
    const viewMissionSpy = spyOn(commandService, 'viewMissionRoom').and.returnValue();
    component.viewMission(getTestRobot(), true);
    expect(viewMissionSpy).toHaveBeenCalled();
  });

  it('handleSimulationRobot should call simulateMissionRobot', () => {
    const simulateMissionSpy = spyOn(commandService, 'simulateMissionRobot').and.returnValue();
    component.handleSimulationRobot(getTestRobot());
    expect(simulateMissionSpy).toHaveBeenCalled();
  });

  it('should return true for sim type if a simulation room with the same robot IP exists', () => {
    component.simulationRooms = [getTestMission()];
    expect(component.isAvailableRoom('sim')).toBeTruthy();
  });

  it('should return false for sim type if no simulation room with the same robot IP exists', () => {
    component.simulationRooms = [];
    expect(component.isAvailableRoom('sim')).toBeFalsy();
  });

  it('should return false for sim type if no simulation room with the same robot IP exists', () => {
    component.simulationRooms = undefined as any;
    expect(component.isAvailableRoom('sim')).toBeFalsy();
  });

  it('should return true if a room with the same robot IP exists', () => {
    let testMission = getTestMission();
    testMission.robot.state = 'Active on mission';
    component.availableRooms = [testMission];
    expect(component.isAvailableRoom(undefined)).toBeTruthy();
  });

  it('should return false if no room with the same robot IP exists', () => {
    component.availableRooms = [];
    expect(component.isAvailableRoom(undefined)).toBeFalsy();
  });

  it('isInSimRoom should return true if the robot is in a simulation room', () => {
    component.simulationRooms = [getTestMission()];
    expect(component.isInSimRoom('test')).toBeTruthy();
  });

  it('ngOnInit should initialize subscriptions', () => {
    spyOn(socketService, 'getBatteryLevel');
    spyOn(socketService, 'getBatteryLevelSim');
    socketService.batteryLevelSimulation.next({ robotId: component.robot.ipAddress, batteryLevel: 80 });
    socketService.isRoomDeleted.next(false);
    socketService.isRoomCreated.next(true);
    socketService.stopBatteryCall.next(true);
    socketService.stopBatteryCallSimulation.next(true);
    component.ngOnInit();
    expect(component.batterySubscription).toBeDefined();
    expect(component.stopBatterySimulationSubscription).toBeDefined();
    expect(component.availableRoomsSubscription).toBeDefined();
    expect(component.availableSimRoomsSubscription).toBeDefined();
    expect(component.roomDeletedSubscription).toBeDefined();
    expect(component.roomCreatedSubscription).toBeDefined();
  });

  it('should call getBatteryLevel when stopBatteryCall emits false', () => {
    spyOn(socketService, 'getBatteryLevel').and.stub();

    component.ngOnInit();
    socketService.stopBatteryCall.next(false);

    expect(socketService.getBatteryLevel).toHaveBeenCalledWith(component.robot);
  });

  it('should call getBatteryLevelSim when stopBatteryCallSimulation emits false', () => {
    spyOn(socketService, 'getBatteryLevelSim').and.stub();

    component.ngOnInit();
    socketService.stopBatteryCallSimulation.next(false);

    expect(socketService.getBatteryLevelSim).toHaveBeenCalledWith(component.robot);
  });

  it('should update simulationRooms when getAvailableSimulatedRoomsInfo emits', () => {
    const mockRooms: MissionRoom[] = [getTestMission()];
    spyOn(socketService, 'getAvailableSimulatedRoomsInfo').and.returnValue(of(mockRooms));

    component.ngOnInit();

    expect(component.simulationRooms).toEqual(mockRooms);
  });

  it('should call getAvailableRooms and set isRoomDeleted to false when isRoomDeleted emits true', fakeAsync(() => {
    spyOn(socketService, 'getAvailableRooms').and.stub();
    const isRoomDeletedSpy = spyOn(socketService.isRoomDeleted, 'next').and.callThrough();

    component.ngOnInit();
    socketService.isRoomDeleted.next(true);
    tick();

    expect(socketService.getAvailableRooms).toHaveBeenCalled();
    expect(isRoomDeletedSpy).toHaveBeenCalledTimes(2);
    expect(isRoomDeletedSpy).toHaveBeenCalledWith(false);
  }));

  it('should call getAvailableRooms and set isRoomCreated to false when isRoomCreated emits true', fakeAsync(() => {
    spyOn(socketService, 'getAvailableRooms').and.stub();
    const isRoomCreatedSpy = spyOn(socketService.isRoomCreated, 'next').and.callThrough();

    component.ngOnInit();
    socketService.isRoomCreated.next(true);
    tick();

    expect(socketService.getAvailableRooms).toHaveBeenCalled();
    expect(isRoomCreatedSpy).toHaveBeenCalledTimes(2);
    expect(isRoomCreatedSpy).toHaveBeenCalledWith(false);
  }));







  it('should return true if any room has other robots', () => {
    const roomsWithOtherRobots: MissionRoom[] = [
      {  hostId: "0.0.0.0",
        robot: getTestRobot(),
        guestId: [],
        otherRobots: [getTestRobot()] },
    ];
    expect(component.bothRobotsUsed(roomsWithOtherRobots)).toBeTruthy();
  });

  it('should return false if no room has other robots', () => {
    const roomsWithoutOtherRobots: MissionRoom[] = [
     getTestMission(),
    ];
    expect(component.bothRobotsUsed(roomsWithoutOtherRobots)).toBeFalsy();
  });

  it('should call setInitialPosition with the result from the dialog', () => {
    const testResult = 'some-location';
    const mockDialogRef = { afterClosed: () => of(testResult) };
    spyOn(dialog, 'open').and.returnValue(mockDialogRef as any);
    const spy= spyOn(socketService, 'setInitialPosition');
    component.setInitialLocation();

    expect(dialog.open).toHaveBeenCalled();
    expect(spy).toHaveBeenCalledWith(component.robot.name, testResult);
  });

  it('should return true if any room has a robot with ipAddress "simulation"', () => {
    const missionRoom: MissionRoom[] = [{
      hostId: "host123",
      robot: { name: 'robot1', ipAddress: 'simulation', state: 'active', batteryLevel: 100 },
      guestId: ["guest456", "guest789"]
    }];
    expect(component.areAllRobotsLaunched(missionRoom)).toBeTruthy();
  });

  it('should return false if no room has a robot with ipAddress "simulation"', () => {
    const missionRoom: MissionRoom[] = [{
      hostId: "host123",
      robot: { name: 'robot1', ipAddress: '192', state: 'active', batteryLevel: 100 },
      guestId: ["guest456", "guest789"]
    }];
    expect(component.areAllRobotsLaunched(missionRoom)).toBeFalsy();
  });

  it('should subscribe to batteryLevelSimulation and update robotSimulationBatteryLevel on simulation', () => {
    const testRobotBatteryInfo: RobotBatteryInfo = {
      robotId: component.robot.ipAddress,
      batteryLevel: 80
    };

    spyOn(socketService.batteryLevelSimulation, 'asObservable').and.returnValue(of(testRobotBatteryInfo));
    component.simulation = true;
    component.ngOnChanges({
      simulation: new SimpleChange(null, component.simulation, true)
    });

    expect(component.robotSimulationBatteryLevel).toBe(testRobotBatteryInfo.batteryLevel);
  });

});

const getTestRobot =(): Robot => ({
  name: "test",
  ipAddress: "0.0.0.0",
  state: "on",
  batteryLevel: 100
});

const getTestMission = (): MissionRoom => ({
  hostId: "0.0.0.0",
  robot: getTestRobot(),
  guestId: []
});
