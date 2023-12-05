import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { MissionPageComponent } from './mission-page.component';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { MissionRoom, RobotBatteryInfo } from 'src/app/interfaces/models';
import { BehaviorSubject, Observable, Subscription } from 'rxjs';
import { Router, RouterModule } from '@angular/router';
import { RouterTestingModule } from '@angular/router/testing';
import { HomePageComponent } from '../home-page/home-page.component';
import { of,Subject } from 'rxjs';
import { SocketTestHelper } from 'src/app/services/client-socket/client-socket.service.spec';
import { Robot } from 'src/app/interfaces/models';


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
   
   allPhysicalRobots: new BehaviorSubject<boolean>(false),
   roomInfo: new Subject<MissionRoom[]>(),
   simulation: new BehaviorSubject<boolean>(false),
   batteryLevelSimulation: new Subject<RobotBatteryInfo>(),
   robots: new Subject<Robot[]>(),
   isHostLeavingRoom: new Subject<boolean>(),
   getMissionRoomInfo(): Observable<MissionRoom[]> {
     return mockSocketService.roomInfo.asObservable();
  }
  
  };


  beforeEach(() => {

    TestBed.configureTestingModule({
      declarations: [MissionPageComponent, HomePageComponent],
      imports: [
        RouterTestingModule,
        RouterModule.forRoot([
            { path: 'home', component:  HomePageComponent },
        ]),
    ],
        providers: [{ provide: SocketService, useValue: mockSocketService }],
    });

    fixture = TestBed.createComponent(MissionPageComponent);
    component = fixture.componentInstance;
    socketService = TestBed.inject(SocketService);
    router = TestBed.inject(Router);
  });

  it('should create the component', () => {
    expect(component).toBeTruthy();
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
    const spyIsHostLeavingRoom=spyOn(socketService.isHostLeavingRoom,"next");
    spyOn(socketService.isHostLeavingRoom, 'asObservable').and.returnValue(of(true));
    component.ngOnInit();
    tick();
    expect(routerSpy).toHaveBeenCalled();
    expect(spyIsHostLeavingRoom).toHaveBeenCalled();
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
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    spyOn(socketService,"getMissionRoomInfo").and.returnValue(of(missionRoom));
    socketService.roomInfo.next(missionRoom);
    component.ngOnInit();


    tick(); 
    expect(component.room).toEqual(missionRoom); 
  }));



  it('should call terminateSimulationRobot and simulation.next on simulation', () => {
    component.simulation = true;
    spyOn(socketService, 'terminateSimulationRobot');
    spyOn(socketService.simulation, 'next');
    component.handleStopMissionClick({name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100});
    expect(socketService.terminateSimulationRobot).toHaveBeenCalledWith({name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100});
    expect(socketService.simulation.next).toHaveBeenCalledWith(false);
  });

  it('should call terminateAllPhysicalRobots on allPhysicalRobots', () => {
    component.allPhysicalRobots = true;
    spyOn(socketService, 'terminateAllPhysicalRobots');
    component.handleStopMissionClick({name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100});
    expect(socketService.terminateAllPhysicalRobots).toHaveBeenCalledWith(component.robots);
  });

  it('should call stopMission on default case', () => {
    spyOn(socketService, 'stopMission');
    component.handleStopMissionClick({name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100});
    expect(socketService.stopMission).toHaveBeenCalledWith({name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100});
  });

  it('should call returnToBaseSimulation on simulation', () => {
    component.room={
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    component.simulation = true;
    spyOn(socketService, 'returnToBaseSimulation');
    component.handleReturnToBase();
    expect(socketService.returnToBaseSimulation).toHaveBeenCalledWith(component.room.robot);
  });

  it('should call returnToBase with all robots on allPhysicalRobots', () => {
    component.allPhysicalRobots = true;
    spyOn(socketService, 'returnToBase');
    component.handleReturnToBase();
    expect(socketService.returnToBase).toHaveBeenCalledWith(component.robots);
  });

  it('should call returnToBase with room.robot on default case', () => {
    component.room={
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    spyOn(socketService, 'returnToBase');
    component.handleReturnToBase();
    expect(socketService.returnToBase).toHaveBeenCalledWith(component.room.robot);
  });

  it('should call returnToBaseSimulation for "Robot 1" with low battery', () => {
    spyOn(socketService, 'returnToBaseSimulation');
    component.handleBatteryLevel('Robot 1', 29); 
    expect(component.robotSimulationBatteryLevel).toBe(29);
    expect(socketService.returnToBaseSimulation).toHaveBeenCalledWith({
      ipAddress: '192.168.0.110',
      name: 'Robot 1'
    });
  });

  it('should call returnToBaseSimulation for "Robot 2" with low battery', () => {
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    component.room=missionRoom;
    spyOn(socketService, 'returnToBaseSimulation');
    component.handleBatteryLevel('Robot 2', 20); 
    expect(component.robotSimulationBatteryLevelRobot2).toBe(20);
    expect(socketService.returnToBaseSimulation).toHaveBeenCalledWith({
      ipAddress: '192.168.0.122',
      name: 'Robot 2'
    });
  });

  it('should call returnToBaseSimulation for Robot object with very low battery', () => {
    const robot: Robot = {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100};
    spyOn(socketService, 'returnToBaseSimulation');
    component.handleBatteryLevel(robot, 25);
    expect(component.robotSimulationBatteryLevel).toBe(25);
    expect(socketService.returnToBaseSimulation).toHaveBeenCalledWith(robot);
  });

  it('should handle battery info for "Robot 1" in simulation', () => {
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    component.room=missionRoom;
    component.room.robot.ipAddress = "simulation";
    spyOn(component, 'handleBatteryLevel');
    const robotBatteryInfo: RobotBatteryInfo = {
      robotId: "192.168.0.110",
      batteryLevel: 50
    };

    component.handleRobotBatteryInfo(robotBatteryInfo);
    expect(component.handleBatteryLevel).toHaveBeenCalledWith("Robot 1", 50);
  });

  it('should handle battery info for "Robot 2" in simulation', () => {
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    component.room=missionRoom;
    component.room.robot.ipAddress = "simulation";
    spyOn(component, 'handleBatteryLevel');
    const robotBatteryInfo: RobotBatteryInfo = {
      robotId: "192.168.0.122",
      batteryLevel: 60 
    };

    component.handleRobotBatteryInfo(robotBatteryInfo);
    expect(component.handleBatteryLevel).toHaveBeenCalledWith("Robot 2", 60);
  });

  it('should handle battery info for non-simulation robot', () => {
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: {name: 'robot1', ipAddress: '192..168.0.1', state: 'simulation', batteryLevel: 100},
      guestId: ["guest456", "guest789"]
    };
    component.room=missionRoom;
    component.room.robot.ipAddress = "192.168.0.100";
    spyOn(component, 'handleBatteryLevel');
    const robotBatteryInfo: RobotBatteryInfo = {
      robotId: "192.168.0.100",
      batteryLevel: 70
    };

    component.handleRobotBatteryInfo(robotBatteryInfo);
    expect(component.handleBatteryLevel).toHaveBeenCalledWith(component.room.robot, 70);
  });

  it('should subscribe to simulation and handle battery level in simulation', () => { 
    const spy= spyOn(component, 'handleRobotBatteryInfo');
    component.ngOnInit();
    socketService.simulation.next(true);
    socketService.batteryLevelSimulation.next({ robotId: '123', batteryLevel: 50 });

    expect(component.simulation).toBeTrue();
    expect(spy).toHaveBeenCalledWith({ robotId: '123', batteryLevel: 50 });

 
  });

  it('should update robots and their battery levels', () => {

    const initialRobot: Robot = { name: 'robot1', ipAddress: '192.168.0.1', state: 'simulation', batteryLevel: 100 };
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: initialRobot,
      guestId: ["guest456", "guest789"]
    };
  
    component.room = missionRoom;
  
    component.ngOnInit();
  
    const newRobots: Robot[] = [
      { name: 'robot1', ipAddress: '192.168.0.1', state: 'active', batteryLevel: 80 },
      { name: 'robot2', ipAddress: '192.168.0.2', state: 'idle', batteryLevel: 60 }
    ];
  
    socketService.robots.next(newRobots);
  
    expect(component.robots).toEqual(newRobots);
    expect(component.room.robot.batteryLevel).toBe(80);
  });

  it('should update the battery level of otherRobots', () => {
    const initialRobot: Robot = { name: 'robot1', ipAddress: '192.168.0.1', state: 'simulation', batteryLevel: 100 };
    const otherRobot: Robot = { name: 'robot2', ipAddress: '192.168.0.2', state: 'idle', batteryLevel: 100 };
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: initialRobot,
      otherRobots: [otherRobot],
      guestId: ["guest456", "guest789"]
    };
  
    component.room = missionRoom;
  
    component.ngOnInit();
  
    const newRobots: Robot[] = [
      { name: 'robot1', ipAddress: '192.168.0.1', state: 'active', batteryLevel: 80 },
      { name: 'robot2', ipAddress: '192.168.0.2', state: 'idle', batteryLevel: 60 } 
    ];
  
    socketService.robots.next(newRobots);
  
    if (component.room.otherRobots && component.room.otherRobots.length > 0) {
      expect(component.room.otherRobots[0].batteryLevel).toBe(60);
    } else {
      fail('otherRobots is undefined or empty');
    }
  });

  it('should call getLogs if robot state is not simulation', () => {
    const otherRobot: Robot = { name: 'robot2', ipAddress: '192.168.0.2', state: 'idle', batteryLevel: 100 };
    const missionRoom: MissionRoom = {
      hostId: "host123",
      robot: { name: 'robot1', ipAddress: '192.168.0.1', state: 'active', batteryLevel: 100 },
      otherRobots: [otherRobot],
      guestId: ["guest456", "guest789"]
    };
   
  
    const mockGetMissionRoomInfo = new BehaviorSubject<MissionRoom>(missionRoom);
    spyOn(socketService, 'getMissionRoomInfo').and.returnValue(mockGetMissionRoomInfo.asObservable());
    spyOn(socketService, 'getLogs');
  
    component.ngOnInit();
  
    mockGetMissionRoomInfo.next(missionRoom);
  
    expect(socketService.getLogs).toHaveBeenCalledWith(missionRoom.robot, component.allPhysicalRobots);
  });
  
  
  



});
