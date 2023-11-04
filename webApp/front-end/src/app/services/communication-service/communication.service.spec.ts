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
          expect(response).toEqual(robots[0].ipAddress);
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/identify?ip=${robots[0].ipAddress}`);
    expect(request.request.method).toBe('GET');
    request.flush(robots[0].ipAddress);

  });

  it('should get all robots', () => {
    service.getRobots().subscribe({
      next: (response: Robot[]) => {
          expect(response).toEqual(robots);
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/robots`);
    expect(request.request.method).toBe('GET');
    request.flush(robots);

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

  it('should terminate a simulation', () => {
    service.terminateSimulation().subscribe({
      next: (response: string) => {
          expect(response).toEqual('Simulation terminated');
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/terminateSim`);
    expect(request.request.method).toBe('GET');
    request.flush('Simulation terminated');

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

  it('should get the ros connection', () => { 
    service.getRosConnection(robots[0]).subscribe({
      next: (response: string) => {
          expect(response).toEqual(robots[0].ipAddress);
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/robot?ip=${robots[0].ipAddress}`);
    expect(request.request.method).toBe('GET');
    request.flush(robots[0].ipAddress);

  });

  it('should launch a mission', () => { 
    service.launchMission(robots[0]).subscribe({
      next: (response: string) => {
          expect(response).toEqual('Mission launched');
      },
      error: fail,
    });

    const request = httpMock.expectOne(`${baseUrl}/launch?ip=${robots[0].ipAddress}`);
    expect(request.request.method).toBe('GET');
    request.flush('Mission launched');

  });
  

});
