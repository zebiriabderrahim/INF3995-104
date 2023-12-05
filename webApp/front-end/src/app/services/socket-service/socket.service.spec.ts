import { TestBed } from '@angular/core/testing';
import { ClientSocketService } from '../client-socket/client-socket.service';
import { HttpClientModule } from '@angular/common/http';
import { Socket } from 'socket.io-client';
import { SocketTestHelper } from '../client-socket/client-socket.service.spec';
import { SocketService } from './socket.service';
import { Router, RouterModule } from '@angular/router';
import { MissionPageComponent } from 'src/app/pages/mission-page/mission-page.component';
import { RouterTestingModule } from '@angular/router/testing';
import { Log, MissionRoom, Robot, RobotMarkerInfo } from 'src/app/interfaces/models';
import { of, Subject } from 'rxjs';
import { CommunicationService } from '../communication-service/communication.service';

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
  let coordinates: RobotMarkerInfo;

  const mockCommunicationService = {
    saveMission: (type: string) => of({}),
  };


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

    coordinates = {
      robotId: '1234',
      position: {x: 0, y: 0, z: 0},
    };


    mockRoom = {
      hostId: '1234',
      robot: robots[0],
      guestId: ['1235','1236'],
    };
    socketHelper = new SocketTestHelper();
    socketServiceMock = new SocketClientServiceMock();
    socketServiceMock.socket = socketHelper as unknown as Socket;
    socketServiceMock.socket.id=  '1234';

    TestBed.configureTestingModule({
      imports: [
          HttpClientModule,
          RouterTestingModule,
          RouterModule.forRoot([
              { path: 'mission', component:  MissionPageComponent },
          ]),
      ],
      providers: [{ provide: ClientSocketService, useValue: socketServiceMock },{provide: CommunicationService, useValue: mockCommunicationService}],
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


  it('should pass true to rosConnectionError ', () => {
    const spy = spyOn(socketService.rosConnectionError, 'next');
    socketHelper.peerSideEmit('rosConnectionError', true);
    expect(spy).toHaveBeenCalledWith(true);
  });


  it('should send viewMissionRoom event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.viewMissionRoom(robots[0],false);
    expect(sendSpy).toHaveBeenCalledWith('viewMissionRoom', robots[0]);
  });

  it('should send stopMission event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.stopMission(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('stopMission', robots[0]);
  });

  it('should send getRobotPos event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.getRobotPos(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('getRobotPos', robots[0].ipAddress);
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
    const roomInfoSpy = spyOn(socketService.roomInfo, 'next');
    socketHelper.peerSideEmit('createdMissionRoom', mockRoom);
    expect(socketService.currentRoom).toEqual(mockRoom);
    expect(roomInfoSpy).toHaveBeenCalledWith(mockRoom);
  });

  it('should handle set simulation subject to true response', () => {
    const spy = spyOn(socketService.isRoomCreated, 'next');
    const simulationSpy = spyOn(socketService.simulation, 'next');
    mockRoom.robot.ipAddress = '192.168.0.sim';
    socketHelper.peerSideEmit('roomCreated', mockRoom);
    expect(spy).toHaveBeenCalledWith(true);
    expect(simulationSpy).toHaveBeenCalledWith(true);
  });

  it('should handle availableRooms response', () => {
    const spy = spyOn(socketService.availableRooms, 'next');
    socketHelper.peerSideEmit('availableRooms',{"rooms": [mockRoom],"simulated":[mockRoom]});
    expect(spy).toHaveBeenCalledWith([mockRoom]);
  });

  it('should handle addedAsViewer response', () => {
    const spy = spyOn(socketService.roomInfo, 'next');
    socketHelper.peerSideEmit('addedAsViewer', mockRoom);
    expect(spy).toHaveBeenCalledWith(mockRoom);
  });

  it('should handle roomDeleted response', () => {
    spyOn(socketService, 'saveMission').and.callFake(() => 'physical');
    const spy = spyOn(socketService.isRoomDeleted, 'next');
    socketHelper.peerSideEmit('roomDeleted', 'physical');
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should handle hostLeftRoom response', () => {
    const spy = spyOn(socketService.isHostLeavingRoom, 'next');
    socketHelper.peerSideEmit('hostLeftRoom');
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should handle navigate to simulation mission when clicked button', () => {
    const routerSpy = spyOn(router, 'navigate');
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.simulateMission();
    expect(sendSpy).toHaveBeenCalledWith('simulateMission');
    expect(routerSpy).toHaveBeenCalledWith(['/mission']);
  });

  it('should terminate simulation', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.terminateSimulation();
    expect(sendSpy).toHaveBeenCalledWith('stopSimulation');
  });

  it('should receive robotPos', () => {
    const spy = spyOn(socketService.robotPos, 'next');
    socketHelper.peerSideEmit('recieveSimRobotPos', coordinates);
    expect(spy).toHaveBeenCalledWith(coordinates);

  });

  it('should stop simulation', () => {
    const spy = spyOn(socketService.simulation, 'next');
    socketHelper.peerSideEmit('stoppedSimulation');
    expect(spy).toHaveBeenCalledWith(false);
  });

  it('should navigate', () => {
    const routerSpy = spyOn(router, 'navigate');
    socketService.navigate("/mission");
    expect(routerSpy).toHaveBeenCalledWith(['/mission']);
  });

  it('should get available simulated rooms', () => {
    spyOn(socketService.simulationRooms, 'asObservable');
    socketService.getAvailableSimulatedRoomsInfo();
    expect(socketService.simulationRooms.asObservable).toHaveBeenCalled();
  });

  it('should send stopSimulationRobot event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.terminateSimulationRobot(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('stopSimulationRobot', robots[0]);
  });

  it('should send getBatteryLevel event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.getBatteryLevel(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('getBatteryLevel', robots[0]);
  });

  it('should simulate Mission Robot', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.simulateMissionRobot(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('simulateMissionRobot', robots[0]);
  });

  it('should stop battery', () => {
    const spy = spyOn(socketService.stopBatteryCall, 'next');
    socketHelper.peerSideEmit('stopBatteryCall', true);
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should update robots when receiving a new robot', () => {
    const spy = spyOn(socketService.robots, 'next');

    const mockRobot = {
      name: 'robot3',
      ipAddress: '192.168.0.3',
      state: 'idle',
      batteryLevel: 80,
    };

    socketHelper.peerSideEmit('robotBattery', mockRobot);

    expect(spy).toHaveBeenCalledWith([...socketService.robots.value, mockRobot]);

  });

  it('should update robots when receiving a robot battery', () => {
    const robots = socketService.robots.value;

    const mockRobot = {
      name: 'Robot 1',
      ipAddress: '192.168.0.110',
      state: 'On',
      batteryLevel: 80,
    };

    const spy = spyOn(socketService.robots, 'next');
    socketHelper.peerSideEmit('robotBattery', mockRobot);
    expect(spy).toHaveBeenCalledWith([mockRobot, robots[1]]);
  });

  it('should save mission', () => {
    const logs: Log[] = [
      {
        type: "system",
        name: "system",
        message: "Mission arrêtée",
        timestamp: "Nov 03 20:59:39"
      },
      {
        type: "system",
        name: "system",
        message: "Mission arrêtée",
        timestamp: "Nov 03 20:59:59"
      }
    ];

    let mission = {
      logs: [{ timestamp: "Nov 03 20:56:23" }, { timestamp: "Nov 03 20:59:39" }],
      map:  [],
      duration: "",
      type: "",
      robots: "physical",
    };
    socketService.currentLogs.next(logs);
    // const spy = spyOn(c, 'send');
    const spy= spyOn(mockCommunicationService, 'saveMission').and.returnValue(of({}));
    socketService.saveMission("simulation","1.07");
    expect(spy).toHaveBeenCalled();
    socketService.saveMission("pyhyical","1.07");
    expect(spy).toHaveBeenCalled();
  });

  it('should send returnToBase event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.returnToBase(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('returnToBase', robots[0]);
  });

  it('should send returnToBase simulation event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.returnToBaseSimulation(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('returnToBaseSimulation', robots[0]);
  });

  it('should send getLogs  event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.getLogs(robots[0],true);
    expect(sendSpy).toHaveBeenCalledWith('getLogs', socketService.robots.getValue());
    socketService.getLogs(robots[0],false);
    expect(sendSpy).toHaveBeenCalledWith('getLogs', robots[0]);
  });

  it('should send getBatteryLevelSim  event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.getBatteryLevelSim(robots[0]);
    expect(sendSpy).toHaveBeenCalledWith('getBatteryLevelSim', robots[0]);
  });

  it('should launch all robots', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    const routerSpy = spyOn(router, 'navigate');
    socketService.launchAllRobots(robots);
    expect(sendSpy).toHaveBeenCalledWith('launchAllRobots', robots);
    expect(routerSpy).toHaveBeenCalledWith(['/mission']);
  });

  it('should send terminateAllPhysicalRobots event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.terminateAllPhysicalRobots(robots);
    expect(sendSpy).toHaveBeenCalledWith('stopAllRobots', robots);
  });

  it('should viewMissionRoomAllRobots', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    const routerSpy = spyOn(router, 'navigate');
    socketService.viewMissionRoomAllRobots();
    expect(sendSpy).toHaveBeenCalledWith('viewAllRobots');
    expect(routerSpy).toHaveBeenCalledWith(['/mission']);
  });

  it('should send setInitialPosition event', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    socketService.setInitialPosition("position", coordinates);
    expect(sendSpy).toHaveBeenCalledWith('setInitialPosition', {name: "position", data: coordinates});
  });

  it('should add logs', () => {
    const spy = spyOn(socketService.currentLogs, 'next');
    socketHelper.peerSideEmit('log');
    expect(spy).toHaveBeenCalled();
  });

  it('should stop battery call', () => {
    const spy = spyOn(socketService.stopBatteryCallSimulation, 'next');
    socketHelper.peerSideEmit('stopBatteryCallSimulation', true);
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should update map', () => {
    const spy = spyOn(socketService.map, 'next');
    socketHelper.peerSideEmit('map', [1,2,3]);
    expect(spy).toHaveBeenCalledWith([1,2,3]);
  });

  it('should update allPhysicalRobots subject', () => {
    const spy = spyOn(socketService.allPhysicalRobots, 'next');
    socketHelper.peerSideEmit('allRobotsConnected',true);
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should update allSimConnected subject', () => {
    const spy = spyOn(socketService.allSimConnected, 'next');
    socketHelper.peerSideEmit('allSimConnected',true);
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('should update batteryLevelSimulation subject', () => {
    const spy = spyOn(socketService.batteryLevelSimulation, 'next');
    socketHelper.peerSideEmit('receiveBatterySim', {robotId: '1234', batteryLevel: 100});
    expect(spy).toHaveBeenCalledWith({robotId: '1234', batteryLevel: 100});
  });

  it('should receiveDistanceSim emit', () => {
    socketService.currentRoom = mockRoom;
    socketService.currentRoom.hostId = "1234";
    socketService.type="simulation";
    const spy = spyOn(socketService, 'saveMission');
    socketHelper.peerSideEmit('receiveDistanceSim', "1.8m");
    expect(spy).toHaveBeenCalledWith("simulation","1.8m");
  });

  it('should receive robot 2d position emit', () => {
    const spy= spyOn(socketService.robotPos, 'next');
    socketHelper.peerSideEmit('recieveSimRobot2Pos', coordinates);
    expect(spy).toHaveBeenCalledWith(coordinates);
  });

  it('should update battery level and execute return to base when batteryLevel is below 30 and executeReturnToBase is false', () => {
    const mockRobot = { name: 'robot1', ipAddress: '192.168.0.4', state: 'idle', batteryLevel: 25 } as Robot;
    const currentRobot = { name: 'robot1', ipAddress: '192.168.0.4', state: 'idle', batteryLevel: 100 } as Robot;
    socketService.robots.next([currentRobot]);
    socketService.currentRoom = mockRoom;

    spyOn(socketService, 'returnToBase');
    socketService.executeReturnToBase = false;

    socketHelper.peerSideEmit('robotBattery', mockRobot);

    expect(socketService.robots.value.length).toBe(1);
    expect(socketService.robots.value[0].batteryLevel).toBe(mockRobot.batteryLevel);
    expect(socketService.returnToBase).toHaveBeenCalledWith(currentRobot);
    expect(socketService.executeReturnToBase).toBeTrue();
  });

  it('should view mission Room in simulation', () => {
    const sendSpy = spyOn(socketServiceMock, 'send');
    const routerSpy = spyOn(router, 'navigate');
    socketService.viewMissionRoom(robots[0],true);
    expect(sendSpy).toHaveBeenCalledWith('viewMissionRoomSimulation', robots[0]);
    expect(routerSpy).toHaveBeenCalledWith(['/mission']);

  });







});
