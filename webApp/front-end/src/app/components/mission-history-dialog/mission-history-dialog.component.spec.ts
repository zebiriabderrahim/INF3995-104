import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MissionHistoryDialog } from './mission-history-dialog.component';

describe('MissionHistoryDialog', () => {
  let component: MissionHistoryDialog;
  let fixture: ComponentFixture<MissionHistoryDialog>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MissionHistoryDialog ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(MissionHistoryDialog);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
