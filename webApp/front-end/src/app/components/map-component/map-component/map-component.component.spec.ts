import { ComponentFixture, TestBed, fakeAsync, tick, flush } from '@angular/core/testing';
import { MapComponentComponent } from './map-component.component';
import { UrlSerializer } from '@angular/router';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { BehaviorSubject, Subject, of } from 'rxjs';

describe('MapComponentComponent', () => {
  let component: MapComponentComponent;
  let fixture: ComponentFixture<MapComponentComponent>;

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
    currentLogs: new BehaviorSubject([]),
    map: new BehaviorSubject([]),
    simulation: new BehaviorSubject({}),
    robotPos: new BehaviorSubject({}),
  };

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MapComponentComponent ],
      imports: [],
      providers: [UrlSerializer,
        { provide: SocketService, useValue: mockSocketService },
      ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MapComponentComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize canvas contexts and subscribe if map is undefined', () => {
    component.map = undefined as any;
    spyOn(component, 'subscribeToMapUpdates');
    spyOn(component, 'subscribeToRobotPositionUpdates');
    component.ngAfterViewInit();
    expect(component.context).toBeDefined();
    expect(component.foregroundContext).toBeDefined();
    expect(component.foregroundContext2).toBeDefined();
    expect(component.subscribeToMapUpdates).toHaveBeenCalled();
    expect(component.subscribeToRobotPositionUpdates).toHaveBeenCalled();
  });

  it('should initialize canvas contexts and update map if map is defined', () => {
    component.map = [1,2,3] as any;
    spyOn(component, 'updateMap');
    component.ngAfterViewInit();
    expect(component.context).toBeDefined();
    expect(component.foregroundContext).toBeDefined();
    expect(component.foregroundContext2).toBeDefined();
    expect(component.updateMap).toHaveBeenCalled();
  });

  it('ngOnDestroy should clear canvas and set map to empty array', () => {
    component.map = [1,2,3] as any;
    spyOn(component, 'clearCanvas');
    component.ngOnDestroy();
    expect(component.clearCanvas).toHaveBeenCalledTimes(2);
    expect(component.map).toEqual([]);
  });

  it('initializeCanvasContexts should initialize canvas contexts and foregrounds', () => {
    component.initializeCanvasContexts();
    expect(component.context).toBeDefined();
    expect(component.foregroundContext).toBeDefined();
    expect(component.foregroundContext2).toBeDefined();
  });

  it('subscribeToMapUpdates should subscribe to map and simulation', () => {
    spyOn(mockSocketService.map, 'asObservable').and.returnValue(of([]));
    spyOn(mockSocketService.simulation, 'asObservable').and.returnValue(of(true));
    component.subscribeToMapUpdates();
    expect(mockSocketService.map.asObservable).toHaveBeenCalled();
    expect(mockSocketService.simulation.asObservable).toHaveBeenCalled();
  });

  it('subscribeToRobotPositionUpdates should subscribe to robotPos and call updateRobot with correct parameters', () => {
    spyOn(mockSocketService.robotPos, 'asObservable').and.returnValue(of({position: {x: 0, y: 0}, robotId: '1'}));
    spyOn(component, 'updateRobot');
    component.subscribeToRobotPositionUpdates();
    expect(mockSocketService.robotPos.asObservable).toHaveBeenCalled();
    expect(component.updateRobot).toHaveBeenCalledWith({x: 0, y: 0, z: 0}, '1', component.foregroundContext);
  });

  it('subscribeToRobotPositionUpdates should subscribe to robotPos and call updateRobot with correct parameters', () => {
    spyOn(mockSocketService.robotPos, 'asObservable').and.returnValue(of({position: {x: 1, y: 0}, robotId: '2'}));
    spyOn(component, 'updateRobot');
    component.subscribeToRobotPositionUpdates();
    expect(mockSocketService.robotPos.asObservable).toHaveBeenCalled();
    expect(component.updateRobot).toHaveBeenCalledWith({x: 1, y: 0, z: 0}, '2', component.foregroundContext);
  });

  it('should update map', fakeAsync(() => {
    spyOn(component, 'drawMap');

    const newMapData = [0, -1, 0, -1];
    const simulation = true;

    component.updateMap(newMapData, simulation);
    tick(20);
    fixture.detectChanges();
    flush();
    expect(component.drawMap).toHaveBeenCalledWith(newMapData, 320, 320);
  }));

  it('should update robot', fakeAsync(() => {
    spyOn(component, 'drawRobot');

    const position = { x: 10, y: 20, z: 0 };
    const robotId = '1';
    const context = component.foregroundContext;

    component.updateRobot(position, robotId, context);
    tick(20);
    fixture.detectChanges();
    flush();
    expect(component.drawRobot).toHaveBeenCalledWith(position, robotId, context);
  }));

  it('should draw map', () => {
    const mapData = [0, -1, 99, -1];
    const width = 2;
    const height = 2;

    component.drawMap(mapData, width, height);
    expect(component.context).toBeDefined();
  });

  it('should draw robot for simulation', () => {
    const position = {x: 0, y: 0};
    const robotId = '1';
    const context = component.foregroundContext;

    component.drawRobot(position as any, robotId, context);
    expect(component.context).toBeDefined();
  });

  it('should draw robot even for physical robots', () => {
    const position = {x: 0, y: 0};
    const robotId = '2';
    const context = component.foregroundContext;
    component.simulation = false;

    component.drawRobot(position as any, robotId, context);
    expect(component.context).toBeDefined();
  });
});
