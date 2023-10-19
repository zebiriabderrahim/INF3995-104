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
  };

  const mockSocketService = {
    createMissionRoom: (robot: Robot) => {},
    viewMissionRoom: (robot: Robot) => {},
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

  it('should call createMissionRoom', () => {
    spyOn(socketService, 'createMissionRoom');
    commandService.createMissionRoom(robot);
    expect(socketService.createMissionRoom).toHaveBeenCalledWith(robot);
  });

  it('should call viewMissionRoom', () => {
    spyOn(socketService, 'viewMissionRoom');
    commandService.viewMissionRoom(robot);
    expect(socketService.viewMissionRoom).toHaveBeenCalledWith(robot);
  });

  it('should call simulateMission', () => {
    spyOn(communicationService, 'simulateMission').and.returnValue(of({}));
    commandService.simulateMission();
    expect(communicationService.simulateMission).toHaveBeenCalled();
  });

  it('should call terminateSimulation', () => {
    spyOn(communicationService, 'terminateSimulation').and.returnValue(of({}));
    commandService.terminateSimulation();
    expect(communicationService.terminateSimulation).toHaveBeenCalled();
  });
});
