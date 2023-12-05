import { AfterViewInit, Component, ElementRef, Input, NgZone, OnDestroy, ViewChild } from '@angular/core';
import { Coordinates, Robot, RobotMarkerInfo } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';

@Component({
  selector: 'app-map-component',
  templateUrl: './map-component.component.html',
  styleUrls: ['./map-component.component.css']
})
export class MapComponentComponent implements AfterViewInit, OnDestroy {
  @ViewChild('mapCanvas') mapCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('foregroundCanvas') foregroundCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('foregroundCanvas2') foregroundCanvas2!: ElementRef<HTMLCanvasElement>;

  @Input() map!: number[];
  context!: CanvasRenderingContext2D;
  foregroundContext!: CanvasRenderingContext2D;
  foregroundContext2!: CanvasRenderingContext2D;
  simulation!: boolean;
  imageData!: ImageData;

  constructor(private socketService: SocketService, private ngZone: NgZone) {}

  ngAfterViewInit() {
    this.initializeCanvasContexts();

    if (this.map === undefined) {
      this.subscribeToMapUpdates();
      this.subscribeToRobotPositionUpdates();
    } else {
      this.updateMap(this.map, this.simulation);
    }
  }

  ngOnDestroy() {
    this.clearCanvas(this.context, this.mapCanvas.nativeElement);
    this.clearCanvas(this.foregroundContext, this.foregroundCanvas.nativeElement);
    this.map = [];
  }

  initializeCanvasContexts() {
    this.context = this.mapCanvas.nativeElement.getContext('2d') as CanvasRenderingContext2D;
    this.foregroundContext = this.foregroundCanvas.nativeElement.getContext('2d') as CanvasRenderingContext2D;
    this.foregroundContext2 = this.foregroundCanvas2.nativeElement.getContext('2d') as CanvasRenderingContext2D;
  }

  subscribeToMapUpdates() {
    // Subscribe to the map updates from the socket service
    this.socketService.map.asObservable().subscribe((newMapData: number[]) => {
      if (newMapData) this.updateMap(newMapData, this.simulation);
    });

    // Subscribe to the simulation status updates from the socket service
    this.socketService.simulation.asObservable().subscribe((bool: boolean) => {
      this.simulation = bool;
    });
  }

  subscribeToRobotPositionUpdates() {
    // Subscribe to the robot position updates from the socket service
    this.socketService.robotPos.asObservable().subscribe((data: RobotMarkerInfo) => {
      const position: Coordinates = { "x": data.position.x , "y": data.position.y, "z": 0 };
      if (data.robotId == "2") {
        this.updateRobot(position, '2', this.foregroundContext2);
      } else {
        this.updateRobot(position, '1', this.foregroundContext);
      }
    });
  }

  updateMap(newMapData: number[], simulation: boolean) {
    // Run the map update outside of Angular zone to optimize performance
    this.ngZone.runOutsideAngular(() => {
      requestAnimationFrame(() => {
        this.drawMap(newMapData, 320, 320)
      });
    });
  }

  updateRobot(position: Coordinates, robotId: string, context: CanvasRenderingContext2D) {
    // Run the robot update outside of Angular zone to optimize performance
    requestAnimationFrame(() => {
        this.drawRobot(position, robotId, context);
    });
  }

  drawMap(mapData: number[], width: number, height: number) {
    // Draw the map based on the provided map data
    this.context.canvas.width = this.foregroundContext.canvas.width = this.foregroundContext2.canvas.width = width;
    this.context.canvas.height = this.foregroundContext.canvas.height = this.foregroundContext2.canvas.height = height;
    const imageData = this.context.createImageData(this.context.canvas.width, this.context.canvas.height);
    const data = new Uint32Array(imageData.data.buffer);

    const greyColor = 0xFF808080;
    const whiteColor = 0xFFFFFFFF;
    const blackColor = 0xFF000000;

    for (let i = 0; i < mapData.length; i++) {
      switch (mapData[i]) {
        case -1:
          data[i] = greyColor;
          break;
        case 0:
          data[i] = whiteColor;
          break;
        default:
          data[i] = blackColor;
          break;
      }
    }

    this.context.putImageData(imageData, 0, 0);
  }

  drawRobot(position: Coordinates, robotId: string, context: CanvasRenderingContext2D) {
    // Draw the robot on the canvas based on its position and ID
    let robotSize = 5;
    let newPosition: Coordinates = position;
    const robotColor = robotId === "1" ? 0xFF00FF00 : 0xFFFF0000;

    let canvasCenterX = context.canvas.width / 2;
    let canvasCenterY = context.canvas.height / 2;

    if (this.simulation) {
      newPosition.x = position.x * 16 + canvasCenterX;
      newPosition.y = position.y * 16 + canvasCenterY
    } else {
      newPosition.x = (position.x + 8) * 20;
      newPosition.y = (position.y + 8) * 20;
    }

    if (!this.imageData) {
      this.imageData = context.createImageData(2 * robotSize + 1, 2 * robotSize + 1);
    }
    const robotData = new Uint32Array(this.imageData.data.buffer);

    for (let i = 0; i < robotData.length; i++) {
      robotData[i] = robotColor;
    }

    this.clearCanvas(context, context.canvas);
    context.putImageData(this.imageData, newPosition.x, newPosition.y);
  }

  clearCanvas(context: CanvasRenderingContext2D, canvasElement: HTMLCanvasElement) {
    // Clear the content of the canvas
    context.clearRect(0, 0, canvasElement.width, canvasElement.height);
  }
}
