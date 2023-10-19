import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RobotComponent } from './robot.component';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { BehaviorSubject, Subject, of } from 'rxjs';
import { NgModule } from '@angular/core';
import { MatDialogModule } from '@angular/material/dialog';
import { Socket } from 'socket.io-client';
import { By } from '@angular/platform-browser';

@NgModule({
  imports: [MatDialogModule],
})
export class DynamicTestModule {}
describe('RobotComponent', () => {
  let component: RobotComponent;
  let fixture: ComponentFixture<RobotComponent>;
  let socketService: SocketService;
  let commandService: CommandService;

  const mockCommandService = {
    getRobots: () => of([]),
    getMissions: () => of([]),
    identifyRobot: (robot: Robot) => of({}),
    simulateMission: () => of({}),
    terminateSimulation: () => of({}),
    createMissionRoom: (robot: Robot) => of({}),
    viewMissionRoom: (robot: Robot) => of({}),
  };

  const mockSocketService = {
    createMissionRoom: (robot: Robot) => {},
    viewMissionRoom: (robot: Robot) => {},
    getAvailableRooms: jasmine.createSpy('getAvailableRooms').and.returnValue(new BehaviorSubject([])),
    getAvailableMissionRoomsInfo: () => of([getTestMission()]),
    isRoomCreated: new Subject<boolean>(),
    isRoomDeleted: new Subject<boolean>(),
    isHostLeavingRoom: new Subject<boolean>(),
    roomInfo: new Subject<MissionRoom>(),
    availableRooms: new Subject<MissionRoom[]>(),
  };

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ RobotComponent ],
      providers: [
        { provide: CommandService, useValue: mockCommandService },
        { provide: SocketService, useValue: mockSocketService },
    ],
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotComponent);
    socketService = TestBed.inject(SocketService);
    commandService = TestBed.inject(CommandService);
    component = fixture.componentInstance;
    component.robot = getTestRobot();
    component.simulation = false;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('ngOnInit should have subscribed to the necessary variables of socketService', () => {
    component.ngOnInit();
    socketService.availableRooms.next([getTestMission()]);
    expect(component.availableRooms).toEqual([getTestMission()]);
    socketService.isRoomDeleted.next(true);
    expect(socketService.getAvailableRooms).toHaveBeenCalled();
    socketService.isRoomCreated.next(true);
    expect(socketService.getAvailableRooms).toHaveBeenCalled();
  });

  it('should have the button Lancer Mission if no room is available and isAvailableRoom should return false', () => {
    component.availableRooms = [];
    fixture.detectChanges();
    const buttons = fixture.debugElement.queryAll(By.css('button'));
    const launchMissionButton = buttons.find((button) =>
      button.nativeElement.textContent.includes('Lancer mission')
    );
    const viewMissionButton = buttons.find((button) =>
      button.nativeElement.textContent.includes('Voir mission')
    );
    expect(component.isAvailableRoom()).toBeFalsy();
    expect(launchMissionButton!.nativeElement.disabled).toBe(false);
    expect(viewMissionButton).toBeUndefined();
  });

  it('should have the button Voir Mission if a room is available and isAvailableRoom should return true', () => {
    component.availableRooms = [getTestMission()];
    fixture.detectChanges();
    const buttons = fixture.debugElement.queryAll(By.css('button'));
    const launchMissionButton = buttons.find((button) =>
      button.nativeElement.textContent.includes('Lancer mission')
    );
    const viewMissionButton = buttons.find((button) =>
      button.nativeElement.textContent.includes('Voir mission')
    );
    expect(component.isAvailableRoom()).toBeTruthy();
    expect(viewMissionButton!.nativeElement.disabled).toBe(false);
    expect(launchMissionButton).toBeUndefined();
  });

  it('launchMission should call createMissionClick', () => {
    const createMissionSpy = spyOn(commandService, 'createMissionRoom').and.returnValue();
    component.launchMission(getTestRobot());
    expect(createMissionSpy).toHaveBeenCalled();
  });

  it('indentifyRobot should call identifyRobot', () => {
    const identifyRobotSpy = spyOn(commandService, 'identifyRobot').and.returnValue();
    component.indentifyRobot(getTestRobot());
    expect(identifyRobotSpy).toHaveBeenCalled();
  });

  it('viewMission should call viewMissionRoom', () => {
    const viewMissionSpy = spyOn(commandService, 'viewMissionRoom').and.returnValue();
    component.viewMission(getTestRobot());
    expect(viewMissionSpy).toHaveBeenCalled();
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
