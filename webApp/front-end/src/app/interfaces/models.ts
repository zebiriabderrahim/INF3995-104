export interface MissionRoom {
    hostId: string;
    robot: Robot;
    guestId?: string[];
    otherRobots?: Robot[];
}

export interface Robot {
    name: string;
    ipAddress: string;
    state: string;
    batteryLevel: number;
}

export interface Log {
  type: string;
  name: string;
  message: string;
  timestamp: string;
}

export interface Coordinates {
  x: number;
  y: number;
  z: number;
}

export interface RobotMarkerInfo {
  robotId: string;
  position: Coordinates;
}

export interface RobotBatteryInfo {
  robotId: string;
  batteryLevel: number;
}

