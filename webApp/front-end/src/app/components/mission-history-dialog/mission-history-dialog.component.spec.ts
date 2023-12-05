import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionHistoryDialog } from './mission-history-dialog.component';
import { MAT_DIALOG_DATA } from '@angular/material/dialog';
import { Log } from 'src/app/interfaces/models';

describe('MissionHistoryDialog', () => {
  let component: MissionHistoryDialog;
  let fixture: ComponentFixture<MissionHistoryDialog>;

  describe('when logs are provided', () => {
    beforeEach(async () => {
      await TestBed.configureTestingModule({
        declarations: [ MissionHistoryDialog ],
        providers: [
          { provide: MAT_DIALOG_DATA, useValue: { logs: [{}], map: [] } }
        ],
      })
      .compileComponents();

      fixture = TestBed.createComponent(MissionHistoryDialog);
      component = fixture.componentInstance;
      fixture.detectChanges();
    });

    it('should initialize logs and set logShown to true', () => {
      expect(component.map).toEqual([]);
    });
  });

  describe('when only map is provided', () => {
    beforeEach(async () => {
      await TestBed.configureTestingModule({
        declarations: [ MissionHistoryDialog ],
        providers: [
          { provide: MAT_DIALOG_DATA, useValue: { map: [] } }
        ],
      })
      .compileComponents();

      fixture = TestBed.createComponent(MissionHistoryDialog);
      component = fixture.componentInstance;
      fixture.detectChanges();
    });

    it('should initialize map and set logShown to false', () => {
      const mockMap:number[] = [];
      expect(component.map).toEqual(mockMap);
      expect(component.logShown).toBeFalse();
      expect(component.logs).toEqual([]);
    });
  });
});
