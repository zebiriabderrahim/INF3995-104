import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionPageComponent } from './mission-page.component';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { FormsModule } from '@angular/forms';
import { NgModule } from '@angular/core';
import { LogBoxComponent } from 'src/app/components/log-box/log-box.component';
import { BehaviorSubject, Subject, Subscription, of } from 'rxjs';
import { Router } from '@angular/router';

@NgModule({
  imports: [FormsModule],
})
export class DynamicTestModule {}
describe('MissionPageComponent', () => {
  let component: MissionPageComponent;
  let fixture: ComponentFixture<MissionPageComponent>;
  let socketService: SocketService;

  const robotTest: Robot = {
    name: "test",
    ipAddress: "0.0.0.0",
    state: "on",
    batteryLevel: 100
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
      declarations: [ MissionPageComponent, LogBoxComponent ],
      imports: [DynamicTestModule],
      providers: [
        { provide: SocketService, useValue: mockSocketService },
    ],
    })
    .compileComponents();

    fixture = TestBed.createComponent(MissionPageComponent);
    socketService = TestBed.inject(SocketService);
    component = fixture.componentInstance;
    component.room = getTestMission();
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('if socketService.isConnected() is false, should reroute to home page', () => {
    spyOn(socketService, 'isConnected').and.returnValue(false);
    component.ngOnInit();
    expect(socketService.router.navigate).toHaveBeenCalledWith(["/home"]);
  });

  it('ngOnInit should have subscribed to the necessary variables of socketService', () => {
    component.room = undefined as any;
    component.ngOnInit();

    socketService.roomInfo.next(getTestMission());
    expect(component.room).not.toBeUndefined();

    socketService.isHostLeavingRoom.next(true);
    expect(socketService.isHostLeavingRoom).toBeTruthy();
    expect(socketService.router.navigate).toHaveBeenCalledWith(["/home"]);
    expect(socketService.getAvailableRooms).toHaveBeenCalled();
  });

  it('handleStopMissionClick should call stopMission', () => {
    spyOn(socketService, 'stopMission');
    component.handleStopMissionClick(getTestRobot());
    expect(socketService.stopMission).toHaveBeenCalledWith(getTestRobot());
  });

  it('ngOnDestroy should unsubscribe from subscriptions', () => {
    const roomSubscriptionSpy = spyOn(component.roomSubscription as jasmine.SpyObj<Subscription>, 'unsubscribe');
    const isHostLeavingRoomSubscriptionSpy = spyOn(
      component.isHostLeavingRoomSubscription as jasmine.SpyObj<Subscription>,
      'unsubscribe'
    );
    component.ngOnDestroy();
    expect(roomSubscriptionSpy).toHaveBeenCalled();
    expect(isHostLeavingRoomSubscriptionSpy).toHaveBeenCalled();
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
