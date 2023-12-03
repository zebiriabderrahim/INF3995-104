import { Component, OnInit } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';
import { Router, NavigationStart, Navigation } from '@angular/router';
import { MissionRoom } from 'src/app/interfaces/models';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';

@Component({
  selector: 'app-robot-editor',
  templateUrl: './robot-editor.component.html',
  styleUrls: ['./robot-editor.component.css']
})
export class RobotEditorComponent implements OnInit {
  files: any;
  filesSubscription: Subscription | undefined;
  selectedFileName: string;
  selectedFileContent: string;
  isLoading: boolean = false;
  isSaving: boolean = false;
  modified: boolean = false;
  password: string = '';
  simulationRooms: MissionRoom[];
  availableRooms: MissionRoom[];
  availableSimRoomsSubscription: Subscription | undefined;
  availableRoomsSubscription: Subscription | undefined;
  history: string[] = [];
  historyIndex: number = -1;

  constructor(private dialog: MatDialog, private commandService: CommandService, private socketService: SocketService, private router: Router) {
    this.files = [];
    this.simulationRooms = [];
    this.availableRooms = [];
    this.selectedFileName = '';
    this.selectedFileContent = '';

  }

  ngOnInit(): void {
    this.openEditorPasswordDialog();
    this.socketService.getAvailableRooms();

    this.availableSimRoomsSubscription = this.socketService.getAvailableSimulatedRoomsInfo().subscribe((rooms: MissionRoom[]) => {
      this.simulationRooms = rooms;
    });

    this.availableRoomsSubscription = this.socketService.getAvailableMissionRoomsInfo().subscribe((rooms: MissionRoom[]) => {
            this.availableRooms = rooms;
    });
  }

  openEditorPasswordDialog() {
    const dialogRef = this.dialog.open(ErrorDialogComponent, {
      width: '300px',
      disableClose: true,
      data: {
        title: 'Entrer mot de passe',
        message: 'Entrer le mot de passe de connection SSH',
        close: 'editer robots',
        auth: true,
      },
    });

    dialogRef.afterClosed().subscribe((password: any) => {
      if (password) {
        this.isLoading = true;
        this.filesSubscription = this.commandService.getRobotFiles(password).subscribe((data) => {
          if (data === 'unauthorized') {
            this.router.navigate(["/home"]);
          }
          this.password = password;
          this.files = data;
          this.isLoading = false;
        });
      }
    });
  }

  onContentChange(newContent: string) {
    if (this.selectedFileName) {
      this.modified = true;
      this.files[this.selectedFileName] = newContent;

      this.updateHistory(newContent);
    }
  }

  updateHistory(newContent: string) {
    if (this.historyIndex === -1 || newContent !== this.history[this.historyIndex]) {
      this.history = [...this.history.slice(0, this.historyIndex + 1), newContent];
      this.historyIndex++;
    }
  }

  onFileSelect(fileName: any) {
    const oldSelectedFileContent = this.selectedFileContent;
    this.selectedFileName = fileName;
    this.selectedFileContent = this.files[fileName] || '';  // Fallback to empty string if file content is not available
    if (oldSelectedFileContent !== fileName) {
      this.history = [this.selectedFileContent];
      this.historyIndex = 0;
    }
    
  }

  onKeyDown(event: KeyboardEvent) {
    if (event.key === 'Tab') {
      if(event.target){ 
        event.preventDefault();

        const target = event.target as HTMLTextAreaElement;
        const start = target.selectionStart;
        const end = target.selectionEnd;

        const updatedContent = 
        this.selectedFileContent.substring(0, start) +
        "\t" +
        this.selectedFileContent.substring(end);

        this.updateHistory(updatedContent);
        this.selectedFileContent = updatedContent;

        setTimeout(() => {
          target.selectionStart = target.selectionEnd = start + 1;
        });
      }
    }
    if (event.ctrlKey && event.key === 'z') {
      if (this.historyIndex > 0) {
        this.historyIndex--;
        this.selectedFileContent = this.history[this.historyIndex];
      }
      event.preventDefault();
    }
  }

  openQuitDialog() {
    this.socketService.getAvailableRooms();
    if(this.modified){
      const dialogRef = this.dialog.open(ErrorDialogComponent, {
        width: '300px',
        data: {
          title: 'Fichiers modifiés',
          message: 'Les modifications effectuées ne seront pas sauvegardées. Voulez-vous quitter?',
          close: 'quitter',
          save: 'sauvegarder',
        },
      });

      dialogRef.afterClosed().subscribe((quit: any) => {
        if (quit === 'quitter') {
          this.router.navigate(["/home"]);
        } else if (quit === 'sauvegarder') {
          if (this.availableRooms.length === 0) this.saveFiles();
          else this.openErrorDialog();
        }
      });
    } else {
      this.router.navigate(["/home"]);
    }
  }

  openSaveDialog() {
    this.socketService.getAvailableRooms();
    if (this.availableRooms.length > 0) {
      this.openErrorDialog();
    } else {
      const dialogRef = this.dialog.open(ErrorDialogComponent, {
        width: '300px',
        data: {
          title: 'Sauvegarder',
          message: 'Voulez-vous sauvegarder les modifications effectuées?',
          close: 'annuler',
          save: 'sauvegarder',
        },
      });

      dialogRef.afterClosed().subscribe((save: any) => {
        if (save === 'sauvegarder') {
          if (this.availableRooms.length > 0) {
            this.openErrorDialog();
          } else this.saveFiles();
        }
      });
    }
  }

  openErrorDialog() {
    this.dialog.open(ErrorDialogComponent, {
      width: '300px',
      data: {
        title: 'Erreur',
        message: 'Impossible de sauvegarder les fichiers. Veuillez terminer les missions en cours.',
        close: 'fermer',
      },
    });
  }

  saveFiles() {
    this.isSaving = true;
    this.commandService.saveRobotFiles(this.password, this.files).subscribe((data) => {
      this.isSaving = false;
      this.modified = false;
    });
  }
  ngOnDestroy(): void {
    this.availableSimRoomsSubscription?.unsubscribe();
    this.availableRoomsSubscription?.unsubscribe();
  }
}
