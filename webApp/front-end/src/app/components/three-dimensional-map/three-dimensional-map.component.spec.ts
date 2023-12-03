import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ThreeDimensionalMapComponent } from './three-dimensional-map.component';

describe('ThreeDimensionalMapComponent', () => {
  let component: ThreeDimensionalMapComponent;
  let fixture: ComponentFixture<ThreeDimensionalMapComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ThreeDimensionalMapComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ThreeDimensionalMapComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
