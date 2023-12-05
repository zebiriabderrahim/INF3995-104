import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MissionHistoryComponent } from './mission-history.component';
import { MAT_DIALOG_DATA, MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { NgModule } from '@angular/core';
import { HttpClientModule } from '@angular/common/http';
import { BehaviorSubject, Subject, Subscription, of } from 'rxjs';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { MissionHistoryDialog } from 'src/app/components/mission-history-dialog/mission-history-dialog.component';

@NgModule({
  imports: [HttpClientModule],
})
export class DynamicTestModule {}
describe('MissionHistoryComponent', () => {
  let component: MissionHistoryComponent;
  let fixture: ComponentFixture<MissionHistoryComponent>;
  let commandService: CommandService;
  let socketService: SocketService;
  let dialogSpy: jasmine.SpyObj<MatDialog>;


  const mockCommandService = {
    getMissions: () => of([]),
    getMissionMap: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
  };

  const mockSocketService = {
    createMissionRoom: (robot: Robot) => {},
    viewMissionRoom: (robot: Robot) => {},
    getAvailableRooms: jasmine.createSpy('getAvailableRooms').and.returnValue(new BehaviorSubject([])),
    getAvailableMissionRoomsInfo: () => of([]),
    getMissionRoomInfo: () => of({}),
    isConnected: () => true,
    stopMission: (robot: Robot) => {},
    router: {
      navigate: jasmine.createSpy('navigate'),
    },
    isRoomCreated: new Subject<boolean>(),
    isRoomDeleted: new Subject<boolean>(),
    isHostLeavingRoom: new Subject<boolean>(),
    roomInfo: new Subject<MissionRoom>(),
    currentRoom: {} as MissionRoom,
  };

  beforeEach(async () => {
    dialogSpy = jasmine.createSpyObj('MatDialog', ['open']);
    await TestBed.configureTestingModule({
      declarations: [ MissionHistoryComponent, MissionHistoryDialog ],
      imports: [DynamicTestModule, MatDialogModule],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: SocketService, useValue: mockSocketService },
        { provide: MatDialog, useValue: dialogSpy },
        { provide: MAT_DIALOG_DATA, useValue: {} }
    ],

    })
    .compileComponents();

    fixture = TestBed.createComponent(MissionHistoryComponent);
    commandService = TestBed.inject(CommandService);
    socketService = TestBed.inject(SocketService);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should open mission overview dialog with correct parameters', () => {
    const mockMission = { name: 'testMission', map: 'testMap' };
    spyOn(commandService, 'getMissionMap').and.returnValue(of([0,0,0,0]));
    component.openMissionOverview(mockMission);
    expect(commandService.getMissionMap).toHaveBeenCalled();
    expect(dialogSpy.open).toHaveBeenCalledWith(MissionHistoryDialog, {
      data: { missionId: mockMission.name, map: [0,0,0,0], duration:undefined,type:undefined,robots:undefined, distance:undefined },
      width: '80%',
      height: '90%'
    });
  });

  it('should open logs overview dialog with correct parameters', () => {
    const mockMission = { name: 'testMission', logs: 'testLogs' };
    component.openLogsOverview(mockMission);
    expect(dialogSpy.open).toHaveBeenCalledWith(MissionHistoryDialog, {
      data: { missionId: mockMission.name, logs: mockMission.logs },
      width: '80%',
      height: '90%'
    });
  });


  it('ngOnInit should subscribe to missions and isHostLeavingRoom', () => {
    spyOn(socketService.isHostLeavingRoom, 'asObservable').and.returnValue(of(true));
    spyOn(commandService, 'getMissions').and.returnValue(of([getTestMission()]));
    component.ngOnInit();

    expect(commandService.getMissions).toHaveBeenCalled();
    expect(socketService.isHostLeavingRoom.asObservable).toHaveBeenCalled();

    socketService.isHostLeavingRoom.next(true);
    expect(socketService.router.navigate).toHaveBeenCalledWith(["/home"]);
  });

  it('should filter missions for "Mode simulation"', () => {
    component.missions = [
        { type: 'Mode simulation' },
        { type: 'Mode robots physiques' },
    ];

    component.showMissionsByMode('Mode simulation');

    expect(component.filteredMissions.length).toBeGreaterThan(0);
    expect(component.filteredMissions.every((mission: Record<string, any>) => mission["type"] === 'Mode simulation')).toBeTrue();
});

it('should filter missions for "Mode robots physiques"', () => {
  component.missions = [
    { type: 'Mode simulation' },
    { type: 'Mode robots physiques' },
  ];

  component.showMissionsByMode('Mode robots physiques');

  expect(component.filteredMissions.length).toBeGreaterThan(0);
  expect(component.filteredMissions.every((mission: Record<string, any>) => mission["type"] === 'Mode robots physiques')).toBeTrue();
});

it('should show all missions for an unknown mode', () => {
  component.missions = [
    { type: 'Mode simulation' },
    { type: 'Mode robots physiques' },
  ];

  component.showMissionsByMode('Unknown Mode');

  expect(component.filteredMissions).toEqual(component.missions);
});

it('should sort missions by distance', () => {
  const mockMissions = [
    { distance: '10.5 meters' },
    { distance: '5.2 meters' },
  ];
  component.missions = mockMissions;

  component.sortMissionsByDistance();

  const sortedDistances = component.filteredMissions.map((mission: Record<string, any>) => mission["distance"]);
  expect(sortedDistances).toEqual(['5.2 meters', '10.5 meters']);
});

it('should sort missions by duration', () => {
  const mockMissions = [
    { duration: '3 minutes 30 seconds' },
    { duration: '2 minutes 45 seconds' },
  ];
  component.missions = mockMissions;

  component.sortMissionsByDuration();

  const sortedDurations = component.filteredMissions.map((mission: Record<string, any>)=> mission["duration"]);
  expect(sortedDurations).toEqual(['2 minutes 45 seconds', '3 minutes 30 seconds']);
});

it('should sort missions by distance', () => {
  const mockMissions = [
    { distance: '10.5 meters' },
    { distance: '5.2 meters' },
  ];
  component.missions = mockMissions;

  component.sortMissionsByDistance();

  const sortedDistances = component.filteredMissions.map((mission: Record<string, any>) => mission["distance"]);
  expect(sortedDistances).toEqual(['5.2 meters', '10.5 meters']);
});

it('should handle missions with missing or invalid distance values', () => {
  const mockMissions = [
    { distance: '10.5 meters' },
    { distance: '' },
    { distance: '7.3 meters' },
  ];
  component.missions = mockMissions;

  component.sortMissionsByDistance();

  const sortedDistances = component.filteredMissions.map((mission: Record<string, any>) => mission["distance"]);
  expect(sortedDistances).toEqual(['','7.3 meters', '10.5 meters']);
});

it('should sort missions by duration', () => {
  const mockMissions = [
    { duration: '3 minutes 30 seconds' },
    { duration: '2 minutes 45 seconds' },
  ];
  component.missions = mockMissions;

  component.sortMissionsByDuration();

  const sortedDurations = component.filteredMissions.map((mission: Record<string, any>) => mission["duration"]);
  expect(sortedDurations).toEqual(['2 minutes 45 seconds', '3 minutes 30 seconds']);
});

it('should handle missions with missing or invalid duration values', () => {
  const mockMissions = [
    { duration: '2 minutes 45 seconds' },
    { duration: '3 minutes 30 seconds' },
    { duration: "" },
    { duration: "" },
  ];
  component.missions = mockMissions;

  component.sortMissionsByDuration();
  const sortedDurations = component.filteredMissions.map((mission: Record<string, any>) => mission["duration"]);
  expect(sortedDurations).toEqual(['2 minutes 45 seconds', '3 minutes 30 seconds', "", ""]);
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
