import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { TestBed } from '@angular/core/testing';
import { CommunicationService } from './communication.service';
import { Robot } from 'src/app/interfaces/models';


describe('CommunicationServiceService', () => {
  let service: CommunicationService;
  let httpMock: HttpTestingController;
  let baseUrl: string;
  let robots: Robot[];
  let missions: string[];
  beforeEach(() => {
      TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
    });
      service = TestBed.inject(CommunicationService);
      httpMock = TestBed.inject(HttpTestingController);
      baseUrl = service['baseUrl'];
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
      missions = ['mission1', 'mission2'];
 });

 afterEach(() => {
  httpMock.verify();
  });


  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should get the robot when identify button is clicked', () => {
    service.identifyRobot(robots[0]).subscribe({
      next: (response: string) => {
          expect(response).toEqual(robots[0].name);
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/identify?robot=${JSON.stringify(robots[0])}`);
    expect(request.request.method).toBe('GET');
    request.flush(robots[0].name);

  });

  it('should get all missions', () => {
    service.getMissions().subscribe({
      next: (response: string[]) => {
          expect(response).toEqual(missions);
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/missions`);
    expect(request.request.method).toBe('GET');
    request.flush(missions);

  });

  it('should get a mission map', () => {
    service.getMissionMap(missions[0]).subscribe({
      next: (response: string) => {
          expect(response).toEqual('map');
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/missionMap?missionName=${missions[0]}`);
    expect(request.request.method).toBe('GET');
    request.flush('map');

  });

  it('should simulate a mission', () => {
    service.simulateMission().subscribe({
      next: (response: string) => {
          expect(response).toEqual('Simulation started');
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/simulate`);
    expect(request.request.method).toBe('GET');
    request.flush('Simulation started');

  });

  it('should return an observable with the provided result', () => {
    const result = 'success';
    const error = new Error('An error occurred');
    const errorHandler = service.handleError('testRequest', result);

    errorHandler(error).subscribe((value) => {
      expect(value).toBe(result);
    });
  });

  it('should return an observable with undefined if no result is provided', () => {
    const error = new Error('An error occurred');
    const errorHandler = service.handleError('testRequest');

    errorHandler(error).subscribe((value) => {
      expect(value).toBeUndefined();
    });
  });

  it('should launch a mission', () => {
    service.launchMission(robots[0]).subscribe({
      next: (response: any) => {
          expect(response).toEqual(robots[0]);
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/launch?robot=${JSON.stringify(robots[0])}`);
    expect(request.request.method).toBe('GET');
    request.flush(robots[0]);

  });

  it('should save a mission', () => {
    service.saveMission(missions[0]).subscribe({
      next: (response: string) => {
          expect(response).toEqual('Mission saved');
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/saveMission`);
    expect(request.request.method).toBe('POST');
    request.flush('Mission saved');

  });

  it('should get robot files', () => {
    const password = "nbiro";
    service.getRobotFiles(password).subscribe({
      next: (response: any) => {
          expect(response).toEqual("got password");
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/robotFiles?password=${password}`);
    expect(request.request.method).toBe('GET');
    request.flush("got password");

  });

  it('should save files', () => {
    const password = "nbiro";
    const files = "files";
    service.saveRobotFiles(password,files).subscribe({
      next: (response: string) => {
          expect(response).toEqual('files saved');
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/saveRobotFiles?password=${password}`);
    expect(request.request.method).toBe('POST');
    request.flush('files saved');

  });

});
