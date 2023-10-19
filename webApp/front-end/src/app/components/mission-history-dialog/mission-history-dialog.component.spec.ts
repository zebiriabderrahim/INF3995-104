import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MissionHistoryDialog } from './mission-history-dialog.component';
import { MAT_DIALOG_DATA, MatDialog } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';

describe('MissionHistoryDialog', () => {
  let component: MissionHistoryDialog;
  let fixture: ComponentFixture<MissionHistoryDialog>;
  let dialog: MatDialog;
  let commandService: jasmine.SpyObj<CommandService>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ MissionHistoryDialog ],
      providers: [
        { provide: MatDialog, useValue: dialog },
        { provide: CommandService, useValue: commandService },
        { provide: MAT_DIALOG_DATA, useValue: {} }
    ],

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
