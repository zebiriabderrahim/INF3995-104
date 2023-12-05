import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { LogBoxComponent } from './log-box.component';
import { NgModule } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { HttpClientModule } from '@angular/common/http';
import { RouterTestingModule } from '@angular/router/testing';

@NgModule({
  imports: [FormsModule],
})
export class DynamicTestModule {}

describe('LogBoxComponent', () => {
  let component: LogBoxComponent;
  let fixture: ComponentFixture<LogBoxComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [DynamicTestModule, HttpClientModule, RouterTestingModule],
      declarations: [LogBoxComponent],
    }).compileComponents();

    fixture = TestBed.createComponent(LogBoxComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('#delayedFilterLogs should filter logs after a delay', fakeAsync(() => {
    component.logs = [
      { type: 'system', name: 'System Log', message: 'System log message', timestamp: '2023-01-01' },
      { type: 'command', name: 'Command Log', message: 'Command log message', timestamp: '2023-01-02' },
      { type: 'other', name: 'Other Log', message: 'Other log message', timestamp: '2023-01-03' },
    ];

    component.systemChecked = false;
    component.commandChecked = false;
    component.otherChecked = false;
    component.delayedFilterLogs();
    tick(1); 
    fixture.detectChanges();
    expect(component.logsShown.length).toEqual(0);

    component.systemChecked = false;
    component.commandChecked = true;
    component.otherChecked = true;
    component.delayedFilterLogs();
    tick(1); 
    fixture.detectChanges();
    expect(component.logsShown.length).toEqual(2);
    expect(component.logsShown.some(log => log.type === 'system')).toBeFalsy();

    component.systemChecked = true;
    component.commandChecked = false;
    component.otherChecked = true;
    component.delayedFilterLogs();
    tick(1); 
    fixture.detectChanges();
    expect(component.logsShown.length).toEqual(2);
    expect(component.logsShown.some(log => log.type === 'command')).toBeFalsy();


    component.systemChecked = true;
    component.commandChecked = true;
    component.otherChecked = false;
    component.delayedFilterLogs();
    tick(1); 
    fixture.detectChanges();
    expect(component.logsShown.length).toEqual(2);
    expect(component.logsShown.some(log => log.type === 'other')).toBeFalsy();

    component.systemChecked = true;
    component.commandChecked = true;
    component.otherChecked = true;
    component.delayedFilterLogs();
    tick(1); 
    fixture.detectChanges();
    expect(component.logsShown.length).toEqual(3);
  }));
});
