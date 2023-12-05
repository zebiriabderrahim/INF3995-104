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

  identifyRobot(robot: Robot): Observable<any> {
    /***
     * GET request to identify the robot
     * @param robot: Robot
     * @return Observable<any>
     */
    return this.http.get<any>(`${this.baseUrl}/identify?robot=${JSON.stringify(robot)}`).pipe(catchError(this.handleError<any>('identifyRobot')));
  }

  launchMission(robot: Robot): Observable<any> {
    /***
     * GET request to launch the mission
     * @param robot: Robot
     * @return Observable<any>
     */
    return this.http.get<any>(`${this.baseUrl}/launch?robot=${JSON.stringify(robot)}`).pipe(catchError(this.handleError<any>('launchMission')));
  }

  getMissions(): Observable<any> {
    /***
     * GET request to get all missions frm database
     * @return Observable<any>
     */
    return this.http.get<any>(`${this.baseUrl}/missions`).pipe(catchError(this.handleError<any>('getMissions')));
  }

  getMissionMap(missionName: string): Observable<any> {
    /***
     * GET request to get the mission map
     * @param missionName: mission name of the map
     * @return Observable<any>
     */
    return this.http.get<any>(`${this.baseUrl}/missionMap?missionName=${missionName}`).pipe(catchError(this.handleError<any>('getMissionMap')));
  }

  simulateMission(): Observable<any>  {
    /***
     * GET request to simulate the mission
     * @return Observable<any>
     */
    return this.http.get<any>(`${this.baseUrl}/simulate`).pipe(catchError(this.handleError<any>('simulateMission')));
  }

  saveMission(mission: any): Observable<any>  {
    /***
     * POST request to save the mission
     * @param mission: mission to be saved
     * @return Observable<any>
     */
    return this.http.post<any>(`${this.baseUrl}/saveMission`, mission).pipe(catchError(this.handleError<any>('saveMission')));
  }

  getRobotFiles(password: string): Observable<any>  {
    /***
     * GET request to get the robot files for the code editor
     * @param password: password of the robot
     */
    return this.http.get<any>(`${this.baseUrl}/robotFiles?password=${password}`).pipe(catchError(this.handleError<any>('getRobotFiles')));
  }

  saveRobotFiles(password: string, files: any): Observable<any>  {
    /***
     * POST request to save the robot files of the code editor
     * @param password: password of the robot
     * @param files: files to be saved
     */
    return this.http.post<any>(`${this.baseUrl}/saveRobotFiles?password=${password}`, {files}).pipe(catchError(this.handleError<any>('saveRobotFiles')));
  }

  handleError<T>(request: string, result?: T): (error: Error) => Observable<T> {
    /***
     * Handle Http operation that failed.
     * @param request: name of the request
     * @param result: optional value to return as the observable result
     * @return Observable<T>
     */
    return () => of(result as T);
  }

}
