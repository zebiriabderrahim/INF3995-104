import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { MissionPageComponent } from './mission-page.component';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { MissionRoom } from 'src/app/interfaces/models';
import { Subscription } from 'rxjs';
import { Router, RouterModule } from '@angular/router';
import { RouterTestingModule } from '@angular/router/testing';
import { HomePageComponent } from '../home-page/home-page.component';
import { of,Subject } from 'rxjs';
import { SocketTestHelper } from 'src/app/services/client-socket/client-socket.service.spec';


class SocketClientServiceMock extends SocketService {
  override connect() {
      return;
  }
}


describe('MissionPageComponent', () => {
  let component: MissionPageComponent;
  let fixture: ComponentFixture<MissionPageComponent>;
  let socketService: SocketService;
  let router: Router;
  let socketServiceMock: SocketClientServiceMock;
  let socketHelper: SocketTestHelper;


  beforeEach(() => {

    TestBed.configureTestingModule({
      declarations: [MissionPageComponent, HomePageComponent],
      imports: [
        RouterTestingModule,
        RouterModule.forRoot([
            { path: 'home', component:  HomePageComponent },
        ]),
    ],
    providers: [{ provide: socketService, useValue: socketServiceMock }],
    });

    fixture = TestBed.createComponent(MissionPageComponent);
    component = fixture.componentInstance;
    socketService = TestBed.inject(SocketService);
    router = TestBed.inject(Router);
  });

  it('should create the component', () => {
    expect(component).toBeTruthy();
  });

  it('should handle stopClickMission', () => { 
    component.simulation = false;
    const spy=spyOn(socketService, 'stopMission');
    component.handleStopMissionClick({name: 'robot1', ipAddress: '192.168.0.1', state: 'idle', batteryLevel: 100});
    expect(spy).toHaveBeenCalled();

    component.simulation = true;
    const spy2=spyOn(component.socketService, 'terminateSimulationRobot');
    const spy3=spyOn(component.socketService.simulation, 'next');
    component.handleStopMissionClick({name: 'robot1', ipAddress: '192..168.0.1', state: 'idle', batteryLevel: 100});
    expect(spy2).toHaveBeenCalled();
    expect(spy3).toHaveBeenCalledWith(false);
  });


  it('should navigate to "/home" when not connected to the socket', () => {
    spyOn(socketService, 'isConnected').and.returnValue(false);
    const routerSpy = spyOn(router, 'navigate');
    component.ngOnInit(); 
    expect(routerSpy).toHaveBeenCalledWith(['/home']);
  });

  it('should left room ', fakeAsync(() => {
    const routerSpy = spyOn(router, 'navigate');
    const spy= spyOn(socketService, 'getAvailableRooms');
    const spyisHostLeavingRoom=spyOn(socketService.isHostLeavingRoom,"next");
    spyOn(socketService.isHostLeavingRoom, 'asObservable').and.returnValue(of(true));
    component.ngOnInit();
    tick();
    expect(routerSpy).toHaveBeenCalled();
    expect(spyisHostLeavingRoom).toHaveBeenCalled();
    expect(spy).toHaveBeenCalled();
  }));


  it('should subscribe to roomSubscription and isHostLeavingRoomSubscription on ngOnInit', () => {
    component.ngOnInit();
    expect(component.roomSubscription).toBeTruthy();
    expect(component.isHostLeavingRoomSubscription).toBeTruthy();
  });

  it('should add info when room is undefined', fakeAsync(() => {
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'idle', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };

    spyOn(socketService,"getMissionRoomInfo").and.returnValue(of(missionRoom));
    socketService.roomInfo.next(missionRoom);
    component.ngOnInit();

    console.log(component.room);

    tick(); 
    expect(component.room).toEqual(missionRoom); 
  }));

  // it('should unsubscribe from subscriptions on ngOnDestroy', fakeAsync(() => {

  //   component.roomSubscription = new Subscription();
  //   component.isHostLeavingRoomSubscription= new Subscription();
    
  //   const spy= spyOn(component.isHostLeavingRoomSubscription, 'unsubscribe');
  //   const spy2= spyOn(component.roomSubscription, 'unsubscribe');

  //   expect(component.roomSubscription).toBeDefined();
  //   expect(component.isHostLeavingRoomSubscription).toBeDefined();

  //   component.ngOnDestroy();
  //   tick();
  //   expect(spy).toHaveBeenCalled();
  //   expect(spy2).toHaveBeenCalled();
  // }));

  // it('should navigate to home and reset isHostLeavingRoom when host is leaving room', fakeAsync(() => {
  //   // Trigger the function you want to test
  //   const spy2= spyOn(router, 'navigate');
    
  //   component.ngOnInit();
  //   // Simulate isHostLeavingRoom emitting true
  //   socketService.isHostLeavingRoom.next(true);
  //   tick(); // Let any asynchronous operations complete

  //   // Check that the component navigated to /home

  //   // Check that isHostLeavingRoom was reset to false
  //   // expect(spy).toHaveBeenCalledWith(false);

  //   // Check that getAvailableRooms was called
  //   expect(socketService.getAvailableRooms).toHaveBeenCalled();
  //   expect(spy2).toHaveBeenCalled();
  // }));



});
