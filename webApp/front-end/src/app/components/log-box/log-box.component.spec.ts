import { ComponentFixture, TestBed } from '@angular/core/testing';

import { LogBoxComponent } from './log-box.component';
import { NgModule } from '@angular/core';
import { FormsModule } from '@angular/forms';

@NgModule({
  imports: [FormsModule],
})
export class DynamicTestModule {}

describe('LogBoxComponent', () => {
  let component: LogBoxComponent;
  let fixture: ComponentFixture<LogBoxComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [DynamicTestModule],
      declarations: [ LogBoxComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(LogBoxComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
