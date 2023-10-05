import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { environment } from 'src/environments/environment';
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { Robot } from 'src/app/interfaces/models';


@Injectable({
  providedIn: 'root'
})
export class CommunicationService {

  private readonly baseUrl: string = environment.serverURL;

  constructor(private readonly http: HttpClient) {}

  getRobots(): Observable<any> { 
    return this.http.get<any>(`${this.baseUrl}/robots`).pipe(catchError(this.handleError<any>('getRobots')));
  }

  identifyRobot(robot: Robot): Observable<any> {
    return this.http.get<any>(`${this.baseUrl}/identify?ip=${robot.ipAddress}`).pipe(catchError(this.handleError<any>('identifyRobot')));
  }

  getMissions(): Observable<any> { 
    return this.http.get<any>(`${this.baseUrl}/missions`).pipe(catchError(this.handleError<any>('getMissions')));
  } 

  simulateMission(): Observable<any>  {
    return this.http.get<any>(`${this.baseUrl}/simulate`).pipe(catchError(this.handleError<any>('simulateMission')));
  }

  terminateSimulation(): Observable<any>  {
    return this.http.get<any>(`${this.baseUrl}/terminateSim`).pipe(catchError(this.handleError<any>('terminateSim')));
  }

  private handleError<T>(request: string, result?: T): (error: Error) => Observable<T> {
    return () => of(result as T);
  }

}
