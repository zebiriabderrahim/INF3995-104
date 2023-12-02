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
    return this.http.get<any>(`${this.baseUrl}/identify?robot=${JSON.stringify(robot)}`).pipe(catchError(this.handleError<any>('identifyRobot')));
  }

  launchMission(robot: Robot): Observable<any> {
    return this.http.get<any>(`${this.baseUrl}/launch?robot=${JSON.stringify(robot)}`).pipe(catchError(this.handleError<any>('launchMission')));
  }

  getMissions(): Observable<any> {
    return this.http.get<any>(`${this.baseUrl}/missions`).pipe(catchError(this.handleError<any>('getMissions')));
  }

  getMissionMap(missionName: string): Observable<any> {
    return this.http.get<any>(`${this.baseUrl}/missionMap?missionName=${missionName}`).pipe(catchError(this.handleError<any>('getMissionMap')));
  }

  simulateMission(): Observable<any>  {
    return this.http.get<any>(`${this.baseUrl}/simulate`).pipe(catchError(this.handleError<any>('simulateMission')));
  }

  terminateSimulation(): Observable<any>  {
    return this.http.get<any>(`${this.baseUrl}/terminateSim`).pipe(catchError(this.handleError<any>('terminateSim')));
  }

  getRosConnection(robot: Robot): Observable<any>  {
    return this.http.get<any>(`${this.baseUrl}/robot?ip=${robot.ipAddress}`).pipe(catchError(this.handleError<any>('getRosConnection')));
  }

  saveMission(mission: any): Observable<any>  {
    return this.http.post<any>(`${this.baseUrl}/saveMission`, mission).pipe(catchError(this.handleError<any>('saveMission')));
  }

  getRobotFiles(password: string): Observable<any>  {
    return this.http.get<any>(`${this.baseUrl}/robotFiles?password=${password}`).pipe(catchError(this.handleError<any>('getRobotFiles')));
  }

  saveRobotFiles(password: string, files: any): Observable<any>  {
    return this.http.post<any>(`${this.baseUrl}/saveRobotFiles?password=${password}`, files).pipe(catchError(this.handleError<any>('saveRobotFiles')));
  }

  handleError<T>(request: string, result?: T): (error: Error) => Observable<T> {
    return () => of(result as T);
  }

}
