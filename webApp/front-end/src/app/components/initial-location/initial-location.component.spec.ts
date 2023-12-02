import { ComponentFixture, TestBed } from '@angular/core/testing';

import { InitialLocationComponent } from './initial-location.component';

describe('InitialLocationComponent', () => {
  let component: InitialLocationComponent;
  let fixture: ComponentFixture<InitialLocationComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ InitialLocationComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(InitialLocationComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
