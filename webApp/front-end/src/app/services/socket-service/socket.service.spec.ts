import { TestBed } from '@angular/core/testing';
import { ClientSocketService } from '../client-socket/client-socket.service';
import { HttpClientModule } from '@angular/common/http';
import { Socket } from 'socket.io-client';
import { SocketTestHelper } from '../client-socket/client-socket.service.spec';
import { SocketService } from './socket.service';
import { Router, RouterModule } from '@angular/router';
import { MissionPageComponent } from 'src/app/pages/mission-page/mission-page.component';
import { RouterTestingModule } from '@angular/router/testing';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { of, Subject } from 'rxjs';

class SocketClientServiceMock extends ClientSocketService {
  override connect() {
      return;
  }
}


describe('SocketService', () => {
  let socketService: SocketService;
  let socketServiceMock: SocketClientServiceMock;
  let router: Router;
  let clientSocket: ClientSocketService;
  let socketHelper: SocketTestHelper;
  let robots: Robot[];
  let mockRoom: MissionRoom;


  beforeEach(() => {

    robots = [{
      name: 'robot1',
      ipAddress: '192.168.0.4',
      state: 'idle',
      batteryLevel: 100,
    },
    {
      name: 'robot2',
      ipAddress: '192.168.0.2',
      state: 'idle',
      batteryLevel: 90,
    }];

    mockRoom = {
      hostId: '1234',
      robot: robots[0],
      guestId: ['1235','1236'],
    };
    socketHelper = new SocketTestHelper();
    socketServiceMock = new SocketClientServiceMock();
    socketServiceMock.socket = socketHelper as unknown as Socket;

    TestBed.configureTestingModule({
      imports: [
          HttpClientModule,
          RouterTestingModule,
          RouterModule.forRoot([
              { path: 'mission', component:  MissionPageComponent },
          ]),
      ],
      providers: [{ provide: ClientSocketService, useValue: socketServiceMock }],
      declarations: [MissionPageComponent],
  });

    socketService = TestBed.inject(SocketService);
    clientSocket = TestBed.inject(ClientSocketService);
    router = TestBed.inject(Router);
    socketService.handleSocket();
  });
  
  it('should be created', () => {
    expect(socketService).toBeTruthy();
  });
  
  it('should not call connect if socket is alive', () => {
    spyOn(socketServiceMock, 'isSocketAlive').and.callFake(() => true);
    const connectSpy = spyOn(socketServiceMock, 'connect');
    socketService.connect();
    expect(connectSpy).not.toHaveBeenCalled();
  });
  
  it('should call connect if socket is not alive', () => {
    spyOn(socketServiceMock, 'isSocketAlive').and.callFake(() => false);
    const connectSpy = spyOn(socketServiceMock, 'connect');
    socketService.connect();
    expect(connectSpy).toHaveBeenCalled();
  });
  
  it('should return true if socket is alive', () => {
    spyOn(socketServiceMock, 'isSocketAlive').and.callFake(() => true);
    expect(socketService.isConnected()).toBeTrue();
  });
  
  it('should return false if socket is not alive', () => {
    spyOn(socketServiceMock, 'isSocketAlive').and.callFake(() => false);
    expect(socketService.isConnected()).toBeFalse();
  });
  
  it('should get mission room info', () => { 
    const spy = spyOn(socketService.roomInfo, 'asObservable');
    socketService.getMissionRoomInfo();
    expect(spy).toHaveBeenCalled();
  });

  it('should send createMissionRoom event', () => {
    const routerSpy = spyOn(router, 'navigate');
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.createMissionRoom(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('createMissionRoom', robots[0]);
    expect(routerSpy).toHaveBeenCalledWith(['/mission']);
  });


  it('should send viewMissionRoom event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.viewMissionRoom(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('viewMissionRoom', robots[0]);
  });

  it('should send stopMission event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.stopMission(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('stopMission', robots[0]);
  });

  it('should return available rooms', () => {
    spyOn(socketService.availableRooms, 'asObservable');
    socketService.getAvailableMissionRoomsInfo();
    expect(socketService.availableRooms.asObservable).toHaveBeenCalled();
    
  });


  it('should send getAvailableRooms event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.getAvailableRooms();
    expect(sendSpy).toHaveBeenCalledWith('getAvailableRooms');
  });
  
  it('should handle createdMissionRoom response', () => {
    const spy = spyOn(socketService.isRoomCreated, 'next');
    const roomInfoSpy = spyOn(socketService.roomInfo, 'next');
    socketHelper.peerSideEmit('createdMissionRoom', mockRoom);
    expect(socketService.currentRoom).toEqual(mockRoom);
    expect(roomInfoSpy).toHaveBeenCalledWith(mockRoom);
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should handle availableRooms response', () => {
    const spy = spyOn(socketService.availableRooms, 'next');
    socketHelper.peerSideEmit('availableRooms', [mockRoom]);
    expect(spy).toHaveBeenCalledWith([mockRoom]);
  });

  it('should handle addedAsViewer response', () => {
    const spy = spyOn(socketService.roomInfo, 'next');
    socketHelper.peerSideEmit('addedAsViewer', mockRoom);
    expect(spy).toHaveBeenCalledWith(mockRoom);
  });

  it('should handle roomDeleted response', () => {
    const spy = spyOn(socketService.isRoomDeleted, 'next');
    socketHelper.peerSideEmit('roomDeleted');
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should handle hostLeftRoom response', () => {
    const spy = spyOn(socketService.isHostLeavingRoom, 'next');
    socketHelper.peerSideEmit('hostLeftRoom');
    expect(spy).toHaveBeenCalledWith(true);
  });

});
