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
    await TestBed.configureTestingModule({
      declarations: [ MissionHistoryComponent, MissionHistoryDialog ],
      imports: [DynamicTestModule, MatDialogModule],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: SocketService, useValue: mockSocketService },
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

  it('should open mission overview dialog', () => {
    const mockMissionData = { name: 'testMission' };
    const dialogSpy = spyOn((component as any).missionHistoryDialog, 'open').and.stub();
    component.missions = [mockMissionData];
    component.openMissionOverview();
    expect(dialogSpy).toHaveBeenCalledWith(MissionHistoryDialog, {
      data: { missionid: mockMissionData.name },
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

  // it('ngOnDestroy should unsubscribe from subscriptions', () => {
  //   const missionsSubscriptionSpy = spyOn(component.missionsSubscription as Subscription, 'unsubscribe');
  //   const isHostLeavingRoomSubscriptionSpy = spyOn(
  //     component.isHostLeavingRoomSubscription as Subscription,
  //     'unsubscribe'
  //   );
  //   component.ngOnDestroy();
  //   expect(missionsSubscriptionSpy).toHaveBeenCalled();
  //   expect(isHostLeavingRoomSubscriptionSpy).toHaveBeenCalled();
  // });
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
