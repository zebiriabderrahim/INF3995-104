import { ComponentFixture, TestBed } from '@angular/core/testing';
import { RobotEditorComponent } from './robot-editor.component';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { RouterTestingModule } from '@angular/router/testing';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';
import { of } from 'rxjs';
import { MissionRoom } from 'src/app/interfaces/models';
import { Router } from '@angular/router';

describe('RobotEditorComponent', () => {
  let component: RobotEditorComponent;
  let fixture: ComponentFixture<RobotEditorComponent>;
  let dialogSpy: jasmine.SpyObj<MatDialog>;
  let commandServiceSpy: jasmine.SpyObj<CommandService>;
  let socketServiceSpy: jasmine.SpyObj<SocketService>;


  beforeEach(async () => {
    dialogSpy = jasmine.createSpyObj('MatDialog', ['open']);
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of() });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    commandServiceSpy = jasmine.createSpyObj('CommandService', ['getRobotFiles', 'saveRobotFiles']);
    commandServiceSpy.getRobotFiles.and.returnValue(of());
    commandServiceSpy.saveRobotFiles.and.returnValue(of());

    socketServiceSpy = jasmine.createSpyObj('SocketService', ['getAvailableSimulatedRoomsInfo', 'getAvailableMissionRoomsInfo']);
    socketServiceSpy.getAvailableSimulatedRoomsInfo.and.returnValue(of());
    socketServiceSpy.getAvailableMissionRoomsInfo.and.returnValue(of());
  
    await TestBed.configureTestingModule({
      declarations: [ RobotEditorComponent ],
      imports: [MatDialogModule, RouterTestingModule],
      providers: [
        { provide: MatDialog, useValue: dialogSpy },
        { provide: CommandService, useValue: commandServiceSpy },
        { provide: SocketService, useValue: socketServiceSpy }
      ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotEditorComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });
  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should open the editor password dialog on initialization', () => {
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of('password') });
    dialogSpy.open.and.returnValue(dialogRefSpy);
  
    component.openEditorPasswordDialog();
    fixture.detectChanges();
  
    expect(dialogSpy.open).toHaveBeenCalledWith(ErrorDialogComponent, {
      width: '300px',
      disableClose: true,
      data: jasmine.any(Object) 
    });
    expect(commandServiceSpy.getRobotFiles).toHaveBeenCalledWith('password');
  });

  it('should subscribe and set simulation rooms on ngOnInit', () => {
    const mockRobot={
      name: 'robot1',
      ipAddress: '192.168.0.4',
      state: 'idle',
      batteryLevel: 100,
    };
    const mockSimulatedRooms: MissionRoom[] = [
      { hostId: '1', robot: mockRobot},
      { hostId: '2', robot: mockRobot},
      { hostId: '3', robot: mockRobot},
    ];
     
    socketServiceSpy.getAvailableSimulatedRoomsInfo.and.returnValue(of(mockSimulatedRooms));
    socketServiceSpy.getAvailableMissionRoomsInfo.and.returnValue(of(mockSimulatedRooms));
    component.ngOnInit();
    expect(component.simulationRooms).toEqual(mockSimulatedRooms);
    expect(socketServiceSpy.getAvailableSimulatedRoomsInfo).toHaveBeenCalled();

    expect(component.availableRooms).toEqual(mockSimulatedRooms);
    expect(socketServiceSpy.getAvailableMissionRoomsInfo).toHaveBeenCalled();
  });

  it('should handle dialog closure and load files if password is provided', () => {
    const mockPassword = 'validPassword';
    const mockFilesData = [{ fileName: 'file1.txt', content: 'Sample Content' }]; 
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of(mockPassword) });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    commandServiceSpy.getRobotFiles.and.returnValue(of(mockFilesData));
  
    component.openEditorPasswordDialog();
    fixture.detectChanges();
  
    expect(dialogSpy.open).toHaveBeenCalled();
    expect(commandServiceSpy.getRobotFiles).toHaveBeenCalledWith(mockPassword);
    expect(component.isLoading).toBeFalse();
    expect(component.password).toEqual(mockPassword);
    expect(component.files).toEqual(mockFilesData);
  });

  it('should navigate to /home if response is unauthorized', () => {
    const mockPassword = 'validPassword';
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of(mockPassword) });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    commandServiceSpy.getRobotFiles.and.returnValue(of('unauthorized'));
    const router = TestBed.inject(Router);
    const navigateSpy = spyOn(router, 'navigate');
  
    component.openEditorPasswordDialog();
    fixture.detectChanges();
  
    expect(dialogSpy.open).toHaveBeenCalled();
    expect(commandServiceSpy.getRobotFiles).toHaveBeenCalledWith(mockPassword);
    expect(navigateSpy).toHaveBeenCalledWith(['/home']);
  });

  it('should update file content and mark as modified when a file is selected', () => {
    component.selectedFileName = 'testFile.txt';
    component.files[component.selectedFileName] = 'Original Content';
    const newContent = 'Updated Content';
    component.onContentChange(newContent);
    expect(component.modified).toBeTrue();
    expect(component.files[component.selectedFileName]).toEqual(newContent);
  });

  it('should not update file content or mark as modified when no file is selected', () => {
    component.selectedFileName = '';
    component.files['testFile.txt'] = 'Original Content';
  
    const newContent = 'Updated Content';
    component.onContentChange(newContent);
  
    expect(component.modified).toBeFalse();
    expect(component.files['testFile.txt']).toEqual('Original Content');
  });
  
  it('should update selected file name and content when file content is available', () => {
    const fileName = 'testFile.txt';
    const fileContent = 'File content';
    component.files = {
      'testFile.txt': fileContent
    };
  
    component.onFileSelect(fileName);

    expect(component.selectedFileName).toEqual(fileName);
    expect(component.selectedFileContent).toEqual(fileContent);
    expect(component.history).toEqual([fileContent]);
    expect(component.historyIndex).toEqual(0);
  });
  
  it('should fallback to empty string for selected file content when file content is not available', () => {
    const fileName = 'emptyFile.txt';
    component.files = {
      'testFile.txt': 'File content'
    };
  
    component.onFileSelect(fileName);  
    expect(component.selectedFileName).toEqual(fileName);
    expect(component.selectedFileContent).toEqual('');
    expect(component.history).toEqual(['']);
    expect(component.historyIndex).toEqual(0);
  });

  it('should insert a tab character on Tab key press', () => {

    component.selectedFileContent = 'Some text';
  
    const mockTextArea = document.createElement('textarea');
    mockTextArea.value = 'Some text';
    mockTextArea.selectionStart = 4;
    mockTextArea.selectionEnd = 4;

    const event = new KeyboardEvent('keydown', { key: 'Tab' });
    spyOn(event, 'preventDefault');

    spyOn(component, 'updateHistory').and.callThrough();

    Object.defineProperty(event, 'target', { writable: false, value: mockTextArea });

    component.onKeyDown(event);

    expect(event.preventDefault).toHaveBeenCalled();
    expect(component.selectedFileContent).toEqual('Some	 text');
    expect(component.updateHistory).toHaveBeenCalledWith('Some	 text');
  });

  it('should update component state with files data if response is valid', () => {
    const mockPassword = 'validPassword';
    const mockFilesData = [{ fileName: 'file1.txt', content: 'Sample Content' }];
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of(mockPassword) });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    commandServiceSpy.getRobotFiles.and.returnValue(of(mockFilesData));
  
    component.openEditorPasswordDialog();
    fixture.detectChanges();
  
    expect(dialogSpy.open).toHaveBeenCalled();
    expect(commandServiceSpy.getRobotFiles).toHaveBeenCalledWith(mockPassword);
    expect(component.isLoading).toBeFalse();
    expect(component.password).toEqual(mockPassword);
    expect(component.files).toEqual(mockFilesData);
  });
  
  it('should revert to previous content on Ctrl + Z key press', () => {
    component.history = ['First content', 'Second content'];
    component.historyIndex = 1;
    const event = new KeyboardEvent('keydown', { key: 'z', ctrlKey: true });
    spyOn(event, 'preventDefault');
  
    component.onKeyDown(event);
  
    expect(component.historyIndex).toEqual(0);
    expect(component.selectedFileContent).toEqual('First content');
    expect(event.preventDefault).toHaveBeenCalled();
  });

  it('should open error dialog with correct parameters', () => {
    component.openErrorDialog();
    expect(dialogSpy.open).toHaveBeenCalledWith(ErrorDialogComponent, {
      width: '300px',
      data: {
        title: 'Erreur',
        message: 'Impossible de sauvegarder les fichiers. Veuillez terminer les missions en cours.',
        close: 'fermer',
      }
    });
  });

  it('should call saveRobotFiles and update state after saving', () => {
    const mockPassword = 'testPassword';
    const mockFiles = { 'file1.txt': 'Content' };
    component.password = mockPassword;
    component.files = mockFiles;
    component.modified = true;
    commandServiceSpy.saveRobotFiles.and.returnValue(of({}));
    component.isSaving = false;
    component.saveFiles();
  
    expect(commandServiceSpy.saveRobotFiles).toHaveBeenCalledWith(mockPassword, mockFiles);
    expect(component.isSaving).toBeFalse();
    expect(component.modified).toBeFalse();
  });

  it('should open error dialog if there are available rooms', () => {
    const robot = {
      name: 'robot1',
      ipAddress: '192.168.0.4',
      state: 'idle',
      batteryLevel: 100,
    };
    const mockRoom = {
      hostId: '1234',
      robot: robot,
      guestId: ['1235','1236'],
    };
    component.availableRooms = [mockRoom, mockRoom];
    spyOn(component, 'openErrorDialog');
  
    component.openSaveDialog();
  
    expect(component.openErrorDialog).toHaveBeenCalled();
  });

  it('should open save dialog and call saveFiles on confirm', () => {
    component.availableRooms = [];
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of('sauvegarder') });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    spyOn(component, 'saveFiles');
  
    component.openSaveDialog();
  
    expect(dialogSpy.open).toHaveBeenCalledWith(ErrorDialogComponent, {
      width: '300px',
      data: {
        title: 'Sauvegarder',
        message: 'Voulez-vous sauvegarder les modifications effectuées?',
        close: 'annuler',
        save: 'sauvegarder',
      }
    });
    expect(component.saveFiles).toHaveBeenCalled();
  });

  it('should open quit dialog and navigate to home on quit', () => {
    component.modified = true;
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of('quitter') });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    const router = TestBed.inject(Router);
    const navigateSpy = spyOn(router, 'navigate');
  
    component.openQuitDialog();
  
    expect(dialogSpy.open).toHaveBeenCalledWith(ErrorDialogComponent, jasmine.any(Object));
    expect(navigateSpy).toHaveBeenCalledWith(['/home']);
  });

  it('should call saveFiles when choosing to save and no available rooms', () => {
    component.modified = true;
    component.availableRooms = [];
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of('sauvegarder') });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    spyOn(component, 'saveFiles');
  
    component.openQuitDialog();
  
    expect(dialogSpy.open).toHaveBeenCalledWith(ErrorDialogComponent, jasmine.any(Object));
    expect(component.saveFiles).toHaveBeenCalled();
  });
  
  it('should open error dialog when choosing to save but rooms are available', () => {
    component.modified = true;
    const robot = {
      name: 'robot1',
      ipAddress: '192.168.0.4',
      state: 'idle',
      batteryLevel: 100,
    };
    const mockRoom = {
      hostId: '1234',
      robot: robot,
      guestId: ['1235','1236'],
    };
    component.availableRooms = [mockRoom];
    const dialogRefSpy = jasmine.createSpyObj({ afterClosed: of('sauvegarder') });
    dialogSpy.open.and.returnValue(dialogRefSpy);
    spyOn(component, 'openErrorDialog');
  
    component.openQuitDialog();
  
    expect(dialogSpy.open).toHaveBeenCalledWith(ErrorDialogComponent, jasmine.any(Object));
    expect(component.openErrorDialog).toHaveBeenCalled();
  });

  it('should navigate to home directly if not modified', () => {
    component.modified = false;
    const router = TestBed.inject(Router);
    const navigateSpy = spyOn(router, 'navigate');
  
    component.openQuitDialog();
  
    expect(navigateSpy).toHaveBeenCalledWith(['/home']);
  });
  
  it('should add new content to history when historyIndex is -1', () => {
    component.history = [];
    component.historyIndex = -1;
    const newContent = 'New content';
  
    component.updateHistory(newContent);
  
    expect(component.history).toEqual([newContent]);
    expect(component.historyIndex).toEqual(0);
  });
  
  it('should add new content to history when it is different from last entry', () => {
    component.history = ['Existing content'];
    component.historyIndex = 0;
    const newContent = 'New content';
  
    component.updateHistory(newContent);
  
    expect(component.history).toEqual(['Existing content', newContent]);
    expect(component.historyIndex).toEqual(1);
  });
  
  
  
  
  
  
  
  
  
  
});
