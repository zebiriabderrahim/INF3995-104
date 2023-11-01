import { Injectable } from '@angular/core';
import { MissionRoom, Robot } from 'src/app/interfaces/models';
import { Router } from '@angular/router';
import { ClientSocketService } from '../client-socket/client-socket.service';
import { BehaviorSubject, Observable, Subject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class SocketService {
  currentRoom!: MissionRoom;
  roomInfo: Subject<MissionRoom>;
  availableRooms: Subject<MissionRoom[]>;
  isRoomDeleted: Subject<boolean>;
  isRoomCreated: Subject<boolean>;
  isHostLeavingRoom: Subject<boolean>;
  robots: BehaviorSubject<Robot[]>;
  stopBatteryCall: BehaviorSubject<Boolean>;


  constructor(private clientSocket: ClientSocketService, public router: Router) {
    this.roomInfo = new Subject<MissionRoom>();
    this.stopBatteryCall = new BehaviorSubject<Boolean>(false);
    this.availableRooms = new Subject<MissionRoom[]>();
    this.isRoomDeleted = new Subject<boolean>();
    this.isRoomCreated = new Subject<boolean>();
    this.isHostLeavingRoom = new Subject<boolean>();  
    this.robots = new BehaviorSubject<Robot[]>([
      {
        name: 'Robot 1',
        ipAddress: '192.168.0.110',
        state: 'Off',
        batteryLevel: 0,
      },
      {
        name: 'Robot 2',
        ipAddress: '192.168.0.122',
        state: 'Off',
        batteryLevel: 0,
      },
    ]);
  }

  connect() {
    if (!this.clientSocket.isSocketAlive()) {
      this.clientSocket.connect();
    }
  }

  isConnected(): boolean {
    return this.clientSocket.isSocketAlive();
  }

  createMissionRoom(robot: Robot) {
    this.clientSocket.send('createMissionRoom', robot);
    this.router.navigate(["/mission"]);
  }

  getMissionRoomInfo(): Observable<MissionRoom> {
    return this.roomInfo.asObservable();
  }

  getAvailableMissionRoomsInfo(): Observable<MissionRoom[]> {
    return this.availableRooms.asObservable();
  }

  getAvailableRooms() {
    this.clientSocket.send('getAvailableRooms');
  }

  viewMissionRoom(robot: Robot) {
    this.clientSocket.send('viewMissionRoom', robot);
    this.router.navigate(["/mission"]);
  }

  stopMission(robot: Robot) {
    this.clientSocket.send('stopMission', robot);
  }

  getBatteryLevel(robotIp: String) {
    this.clientSocket.send('getBatteryLevel', robotIp);
  }

  handleSocket() {
    this.clientSocket.on('createdMissionRoom', (room: any) => {
      this.currentRoom = room;
      this.roomInfo.next(this.currentRoom);
      this.isRoomCreated.next(true);
    });

    this.clientSocket.on('availableRooms', (rooms: MissionRoom[]) => { 
      this.availableRooms.next(rooms);
    });

    this.clientSocket.on('addedAsViewer', (room: MissionRoom) => {
      this.roomInfo.next(room);
    });
    
    this.clientSocket.on('roomDeleted', () => {
      this.isRoomDeleted.next(true);
    })

    this.clientSocket.on('hostLeftRoom', () => {
      this.isHostLeavingRoom.next(true);
    }); 

    this.clientSocket.on('stopBatteryCall', (bool: Boolean) =>{
      this.stopBatteryCall.next(bool);
    })

    this.clientSocket.on('robotBattery', (robot: Robot) => {
      if(this.robots.getValue().length === 0 || !this.robots.value.find((r: Robot) => r.ipAddress === robot.ipAddress)){
        this.robots.next([...this.robots.value, robot]);
      }
      else{
        this.robots.next(this.robots.value.map((robot_old: Robot) => {
          if(robot_old.ipAddress === robot.ipAddress && robot_old.batteryLevel != robot.batteryLevel){
            return {
              ...robot_old,
              state: "On",
              batteryLevel: robot.batteryLevel
            }
          }
          else{
            return robot_old;
          }
        }));
      }
    });
  }
}
