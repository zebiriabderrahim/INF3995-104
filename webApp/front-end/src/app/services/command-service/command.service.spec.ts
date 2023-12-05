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
    getMissions: () => of([]),
    getMissionMap: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
    launchMission: (robot: Robot) => of({}),
    getRobotFiles:(password:string)=>of({}),
    saveRobotFiles:(password:string,files:File[])=>of({}),
  };

  const mockSocketService = {
    createMissionRoom: (robot: Robot) => {},
    simulateMission: () => {},
    viewMissionRoom: (robot: Robot) => {},
    simulateMissionRobot: (robot: Robot) => {},
    launchAllRobots: (robots: Robot[]) => {},
    viewMissionRoomAllRobots: () => {},
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

  it('should call getMissions', () => {
    spyOn(communicationService, 'getMissions').and.returnValue(of([]));
    commandService.getMissions();
    expect(communicationService.getMissions).toHaveBeenCalled();
  });

  it('should call getMissionMap', () => {
    spyOn(communicationService, 'getMissionMap').and.returnValue(of([]));
    commandService.getMissionMap("missionName");
    expect(communicationService.getMissionMap).toHaveBeenCalledWith("missionName");
  });

  it('should call identifyRobot', () => {
    spyOn(communicationService, 'identifyRobot').and.returnValue(of(""));
    commandService.identifyRobot(robot);
    expect(communicationService.identifyRobot).toHaveBeenCalledWith(robot);
  });

  it('should call socketService simulateMissionRobot', () => {
    const spy= spyOn(socketService, 'simulateMissionRobot');
    commandService.simulateMissionRobot(robot);
    expect(spy).toHaveBeenCalledWith(robot);
  });


  it('should call viewMissionRoom', () => {
    spyOn(socketService, 'viewMissionRoom');
    commandService.viewMissionRoom(robot, true);
    expect(socketService.viewMissionRoom).toHaveBeenCalledWith(robot, true);
  });

  it('should call simulateMission', () => {
    spyOn(socketService, 'simulateMission');
    commandService.simulateMission();
    expect(socketService.simulateMission).toHaveBeenCalled();
  });

  it('should call createMissionRoom', () => {
    const spy=spyOn(socketService, 'createMissionRoom');
    commandService.createMissionRoom(robot);
    expect(spy).toHaveBeenCalledWith(robot);
  });

  it('should call launchAllRobots', () => {
    const spy=spyOn(socketService, 'launchAllRobots');
    commandService.launchAllRobots([robot]);
    expect(spy).toHaveBeenCalledWith([robot]);
  });

  it('should call viewMissionRoomAllRobots', () => {
    const spy=spyOn(socketService, 'viewMissionRoomAllRobots');
    commandService.viewMission();
    expect(spy).toHaveBeenCalled();
  });

  it('should call getRobotFiles in communication service', () => {
    const spy=spyOn(communicationService, 'getRobotFiles');
    commandService.getRobotFiles("nvid");
    expect(spy).toHaveBeenCalled();
  });

  it('should call saveRobotFiles in communication service', () => {
    const spy=spyOn(communicationService, 'saveRobotFiles');
    commandService.saveRobotFiles("nvid",[]);
    expect(spy).toHaveBeenCalled();
  });
});
