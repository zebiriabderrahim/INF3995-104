import { Component, OnInit } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { CommandService } from 'src/app/services/command-service/command.service';
import { SocketService } from 'src/app/services/socket-service/socket.service';
import { Subscription } from 'rxjs';
import { Router } from '@angular/router';
import { MissionRoom } from 'src/app/interfaces/models';
import { ErrorDialogComponent } from 'src/app/components/error-dialog/error-dialog.component';

@Component({
  selector: 'app-robot-editor',
  templateUrl: './robot-editor.component.html',
  styleUrls: ['./robot-editor.component.css']
})
export class RobotEditorComponent implements OnInit {
  // Properties for handling file-related information
  files: any;
  filesSubscription: Subscription | undefined;
  selectedFileName: string;
  selectedFileContent: string;
  isLoading: boolean = false;
  isSaving: boolean = false;
  modified: boolean = false;
  password: string = '';

  // Properties for handling mission rooms
  simulationRooms: MissionRoom[];
  availableRooms: MissionRoom[];
  availableSimRoomsSubscription: Subscription | undefined;
  availableRoomsSubscription: Subscription | undefined;

  // Properties for handling editing history
  history: string[] = [];
  historyIndex: number = -1;

  // Constructor to initialize services and default values
  constructor(
    private dialog: MatDialog,
    private commandService: CommandService,
    private socketService: SocketService,
    private router: Router
  ) {
    this.files = [];
    this.simulationRooms = [];
    this.availableRooms = [];
    this.selectedFileName = '';
    this.selectedFileContent = '';
  }

  ngOnInit(): void {
    // Open the password dialog when the editor component is initialized
    this.openEditorPasswordDialog();

    // Request available rooms from the socket service
    this.socketService.getAvailableRooms();

    // Subscribe to updates on available simulated rooms
    this.availableSimRoomsSubscription = this.socketService
      .getAvailableSimulatedRoomsInfo()
      .subscribe((rooms: MissionRoom[]) => {
        this.simulationRooms = rooms;
      });

    // Subscribe to updates on available mission rooms
    this.availableRoomsSubscription = this.socketService
      .getAvailableMissionRoomsInfo()
      .subscribe((rooms: MissionRoom[]) => {
        this.availableRooms = rooms;
      });
  }

  // Open the password dialog for SSH connection
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

  // Handle content change in the editor
  onContentChange(newContent: string) {
    if (this.selectedFileName) {
      this.modified = true;
      this.files[this.selectedFileName] = newContent;

      // Update the editing history
      this.updateHistory(newContent);
    }
  }

  // Update the editing history with new content
  updateHistory(newContent: string) {
    if (this.historyIndex === -1 || newContent !== this.history[this.historyIndex]) {
      this.history = [...this.history.slice(0, this.historyIndex + 1), newContent];
      this.historyIndex++;
    }
  }

  // Handle file selection in the editor
  onFileSelect(fileName: any) {
    const oldSelectedFileContent = this.selectedFileContent;
    this.selectedFileName = fileName;
    this.selectedFileContent = this.files[fileName] || '';

    // Reset history if a different file is selected
    if (oldSelectedFileContent !== fileName) {
      this.history = [this.selectedFileContent];
      this.historyIndex = 0;
    }
  }

  // Handle keydown events in the editor
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

    // Handle undo (Ctrl + Z) functionality
    if (event.ctrlKey && event.key === 'z') {
      if (this.historyIndex > 0) {
        this.historyIndex--;
        this.selectedFileContent = this.history[this.historyIndex];
      }
      event.preventDefault();
    }
  }

  // Open a dialog when attempting to quit the editor
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


  // Open a dialog to confirm saving changes
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

  // Open an error dialog when saving is not possible
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

  // Save the modified files
  saveFiles() {
    this.isSaving = true;
    this.commandService.saveRobotFiles(this.password, this.files).subscribe((data) => {
      this.isSaving = false;
      this.modified = false;
    });
  }

  // Unsubscribe from subscriptions to prevent memory leaks
  ngOnDestroy(): void {
    this.availableSimRoomsSubscription?.unsubscribe();
    this.availableRoomsSubscription?.unsubscribe();
  }
}
