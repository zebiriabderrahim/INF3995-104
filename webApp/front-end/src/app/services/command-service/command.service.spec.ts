import { TestBed, inject } from '@angular/core/testing';
import { CommandService } from './command.service';
import { CommunicationService } from '../communication-service/communication.service';
import { SocketService } from '../socket-service/socket.service';
import { Robot } from 'src/app/interfaces/models';
import { of } from 'rxjs';

describe('CommandService', () => {
  let commandService: CommandService;
  let communicationService: CommunicationService;
  let socketService: SocketService;
  let robot: Robot;


  const mockCommunicationService = {
    getRobots: () => of([]),
    getMissions: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
    terminateSimulation: () => of({}),
    getRosConnection: (robot: Robot) => of({}),
    launchMission: (robot: Robot) => of({}),
  };

  const mockSocketService = {
    createMissionRoom: (robot: Robot) => {},
    simulateMission: () => {},
    viewMissionRoom: (robot: Robot) => {},
    simulateMissionRobot: (robot: Robot) => {},
  };

  beforeEach(() => {
    robot ={
      name: 'robot1',
      ipAddress: '192.168.0.4',
      state: 'idle',
      batteryLevel: 100,
    };

    TestBed.configureTestingModule({
      providers: [
        CommandService,
        { provide: CommunicationService, useValue: mockCommunicationService },
        { provide: SocketService, useValue: mockSocketService },
      ],
    });

    commandService = TestBed.inject(CommandService);
    communicationService = TestBed.inject(CommunicationService);
    socketService = TestBed.inject(SocketService);
  });

  it('should be created', () => {
    expect(commandService).toBeTruthy();
  });

  it('should call getRobots', () => {
    spyOn(communicationService, 'getRobots').and.returnValue(of([]));
    commandService.getRobots();
    expect(communicationService.getRobots).toHaveBeenCalled();
  });

  it('should call getMissions', () => {
    spyOn(communicationService, 'getMissions').and.returnValue(of([]));
    commandService.getMissions();
    expect(communicationService.getMissions).toHaveBeenCalled();
  });

  it('should call identifyRobot', () => {
    spyOn(communicationService, 'identifyRobot').and.returnValue(of(""));
    commandService.identifyRobot(robot);
    expect(communicationService.identifyRobot).toHaveBeenCalledWith(robot);
  });

  it('should call communicationService launchMission', () => {
    spyOn(communicationService, 'launchMission').and.returnValue(of({}));
    commandService.launchMission(robot);
    expect(communicationService.launchMission).toHaveBeenCalledWith(robot);
  });

  it('should call socketService simulateMissionRobot', () => {
    const spy= spyOn(socketService, 'simulateMissionRobot');
    commandService.simulateMissionRobot(robot);
    expect(spy).toHaveBeenCalledWith(robot);
  });


  it('should call viewMissionRoom', () => {
    spyOn(socketService, 'viewMissionRoom');
    commandService.viewMissionRoom(robot);
    expect(socketService.viewMissionRoom).toHaveBeenCalledWith(robot);
  });

  it('should call terminateSimulation', () => {
    spyOn(communicationService, 'terminateSimulation').and.returnValue(of({}));
    commandService.terminateSimulation();
    expect(communicationService.terminateSimulation).toHaveBeenCalled();
  });

  it('should call getRosConnection', () => {
    spyOn(communicationService, 'getRosConnection').and.returnValue(of({}));
    commandService.getRosConnection(robot);
    expect(communicationService.getRosConnection).toHaveBeenCalledWith(robot);
  });

  it('should call simulateMission', () => {
    spyOn(socketService, 'simulateMission');
    commandService.simulateMission();
    expect(socketService.simulateMission).toHaveBeenCalled();
  });

  it('should call launchMission + createMissionRoom', () => {
    const firstSpy=spyOn(commandService, 'launchMission');
    const secondSpy=spyOn(socketService, 'createMissionRoom');
    commandService.createMissionRoom(robot);
    expect(firstSpy).toHaveBeenCalledWith(robot);
    expect(secondSpy).toHaveBeenCalledWith(robot);
  });
});
