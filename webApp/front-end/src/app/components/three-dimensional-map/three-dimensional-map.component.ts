import { Component, OnInit } from '@angular/core';
//import * as THREE from 'three';

@Component({
  selector: 'app-three-dimensional-map',
  templateUrl: './three-dimensional-map.component.html',
  styleUrls: ['./three-dimensional-map.component.css']
})
export class ThreeDimensionalMapComponent implements OnInit {
  // private scene!: THREE.Scene;
  // private camera!: THREE.PerspectiveCamera;
  // private renderer!: THREE.WebGLRenderer;

  constructor() {}

  ngOnInit(): void {
    this.initThree();
    this.animate();
  }

  private initThree(): void {
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    const mapContainer = document.getElementById('mapContainer');
    mapContainer!.appendChild(this.renderer.domElement);
    
    const geometry = new THREE.BoxGeometry();
    const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const cube = new THREE.Mesh(geometry, material);
    this.scene.add(cube);
    this.camera.position.z = 5;
  }

  private animate(): void {
    requestAnimationFrame(() => this.animate());
    this.renderer.render(this.scene, this.camera);
  }

}

// import { Component, OnInit } from '@angular/core';
// import { extend, NgtBeforeRenderEvent, NgtStore } from 'angular-three';
// import * as THREE from 'three';
// import { OrbitControls } from 'three-stdlib';

// extend(THREE);
// extend({ OrbitControls });

// @Component({
//   selector: 'app-three-dimensional-map',
//   template: `
//     <ngt-ambient-light [intensity]="0.5" />
//     <ngt-spot-light [position]="10" [angle]="0.15" [penumbra]="1" />
//     <ngt-point-light [position]="-10" />

//     <ngt-mesh 
//       (beforeRender)="onBeforeRender($event)"
//       [scale]="active ? 1.5 : 1"
//       [position]="[0, 0, 0]"
//     >
//       <ngt-box-geometry />
//       <ngt-mesh-standard-material [color]="hovered ? 'darkred' : 'red'" />
//     </ngt-mesh>

//     <ngt-mesh 
//       (beforeRender)="onBeforeRender($event)"
//       [scale]="active ? 1.5 : 1"
//       [position]="[1.5, 0, 0]"
//     >
//       <ngt-box-geometry />
//       <ngt-mesh-standard-material [color]="hovered ? 'darkred' : 'red'" />
//     </ngt-mesh>

//     <ngt-mesh 
//       (beforeRender)="onBeforeRender($event)"
//       [scale]="active ? 1.5 : 1"
//       [position]="[-1.5, 0, 0]"
//     >
//       <ngt-box-geometry />
//       <ngt-mesh-standard-material [color]="hovered ? 'darkred' : 'red'" />
//     </ngt-mesh>

//     <ngt-orbit-controls *args="[camera, renderer.domElement]" [enableDamping]="true" (beforeRender)="updateControls()" />
//   `,
// })
// export class ThreeDimensionalMapComponent implements OnInit {
//   private scene!: THREE.Scene;
//   private camera!: THREE.PerspectiveCamera;
//   private renderer!: THREE.WebGLRenderer;

//   active = false;
//   hovered = false;

//   constructor(private readonly store: NgtStore) {}

//   ngOnInit(): void {
//     this.initThree();
//     this.animate();
//   }

//   private initThree(): void {
//     this.scene = new THREE.Scene();
//     this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
//     this.renderer = new THREE.WebGLRenderer();
//     this.renderer.setSize(window.innerWidth, window.innerHeight);
//     const mapContainer = document.getElementById('mapContainer');
//     mapContainer!.appendChild(this.renderer.domElement);

//     // Add cubes to the scene
//     const cube1 = this.createCube([0, 0, 0]);
//     const cube2 = this.createCube([1.5, 0, 0]);
//     const cube3 = this.createCube([-1.5, 0, 0]);

//     this.scene.add(cube1, cube2, cube3);
//     this.camera.position.z = 5;
//   }

//   private createCube(position: number[]): THREE.Mesh {
//     const geometry = new THREE.BoxGeometry();
//     const material = new THREE.MeshStandardMaterial({ color: 'red' });
//     const cube = new THREE.Mesh(geometry, material);
//     cube.position.fromArray(position);
//     return cube;
//   }

//   private animate(): void {
//     requestAnimationFrame(() => this.animate());
//     this.renderer.render(this.scene, this.camera);
//   }

//   private updateControls(): void {
//     // Update controls if needed
//   }

//   onBeforeRender(event: NgtBeforeRenderEvent<THREE.Mesh>) {
//     // Perform actions before rendering
//     event.object.rotation.x += 0.01;
//   }
// }

