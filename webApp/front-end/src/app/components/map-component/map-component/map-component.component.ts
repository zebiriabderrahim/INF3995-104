import { AfterViewInit, Component, ElementRef, Input, NgZone, OnInit, ViewChild } from '@angular/core';
import { Subscription } from 'rxjs';
import { Coordinates, Robot, RobotMarkerInfo } from 'src/app/interfaces/models';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import * as THREE from 'three';

@Component({
  selector: 'app-map-component',
  templateUrl: './map-component.component.html',
  styleUrls: ['./map-component.component.css']
})
export class MapComponentComponent implements AfterViewInit {
  @ViewChild('mapCanvas') mapCanvas!: ElementRef<HTMLCanvasElement>;
  private context!: CanvasRenderingContext2D;

  constructor(private ngZone: NgZone, private socketService: SocketService) {}

  ngAfterViewInit() {
    const context = this.mapCanvas.nativeElement.getContext('2d');
    if (!context) {
      throw new Error('CanvasRenderingContext2D is not supported by this browser.');
    }
    this.context = context;
    this.socketService.map.subscribe(
      (newMapData) => {
        this.updateMap(newMapData);
      },
      (error) => {
        console.error('Error receiving map data:', error);
      }
    );
  }

  updateMap(newMapData: number[]) {
    this.ngZone.runOutsideAngular(() => {
      requestAnimationFrame(() => {
        this.drawMap(newMapData);
      });
    });
  }

  private drawMap(mapData: number[]) {
    const width = 1984;
    const height = 1984;
    const imageData = this.context.createImageData(width, height);
    const data = new Uint32Array(imageData.data.buffer);

    // Assuming the endianness is Little Endian, ABGR format is used
    const greyColor = 0xFF808080;  // Grey, with full alpha
    const whiteColor = 0xFFFFFFFF; // White, with full alpha
    const blackColor = 0xFF000000; // Black, with full alpha

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
  // @Input() robot!: Robot;
  // @Input() simulation !: boolean;
  // private renderer!: THREE.WebGLRenderer;
  // private scene!: THREE.Scene;
  // private camera!: THREE.PerspectiveCamera;
  // private robotMarker: THREE.Mesh;
  // private secondRobotMarker: THREE.Mesh;
  // private width = this.elementRef.nativeElement.clientWidth;
  // private height = this.elementRef.nativeElement.clientHeight;

  // constructor(private elementRef: ElementRef, private ngZone: NgZone, private socketService: SocketService) {
  //   const coneGeometry = new THREE.ConeGeometry(0.2, 0.5, 10);
  //   const coneMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
  //   const secondConeMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
  //   this.robotMarker = new THREE.Mesh(coneGeometry, coneMaterial);
  //   this.secondRobotMarker = new THREE.Mesh(coneGeometry, secondConeMaterial);

  //   this.robotMarker.scale.set(2, 2, 2);
  //   this.secondRobotMarker.scale.set(2, 2, 2); 

  //   this.renderer = new THREE.WebGLRenderer();
  //   this.scene = new THREE.Scene();
  //   this.camera = new THREE.PerspectiveCamera(75, this.width / this.height, 0.1, 1000);

  //   this.camera.position.set(0, 0, 30);
  //   this.camera.lookAt(0, 0, 0);
  // }

  // ngOnInit(): void {
  //   this.ngZone.runOutsideAngular(() => {
  //     this.elementRef.nativeElement.appendChild(this.renderer.domElement);
  //     this.onWindowResize();
  //   });

  //   this.render();

  //   this.socketService.robotPos.asObservable().subscribe((coordinates: RobotMarkerInfo) => {
  //     if (coordinates.robotId == '2')  {
  //       this.secondRobotMarker.position.set(coordinates.position.x, coordinates.position.y, coordinates.position.z);
  //       this.scene.add(this.secondRobotMarker);
  //     }
  //     else {
  //       this.robotMarker.position.set(coordinates.position.x, coordinates.position.y, coordinates.position.z);
  //       this.scene.add(this.robotMarker);
  //     }
  //     this.render();
  //   })
  // }

  // onWindowResize() {
  //   this.width = this.elementRef.nativeElement.clientWidth;
  //   this.height = this.elementRef.nativeElement.clientHeight;
  //   this.renderer.setSize(this.width, this.height);
  //   this.camera.aspect = this.width / this.height;
  //   this.camera.updateProjectionMatrix();
  //   this.render();
  // }

  // render() {
  //   this.renderer.render(this.scene, this.camera);
  // }
}
