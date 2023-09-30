export interface MissionRoom {
    hostId: string;
    robot: Robot;
    guestId?: string[];
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
