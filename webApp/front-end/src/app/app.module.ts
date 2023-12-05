import { NgModule } from '@angular/core';
import { MatFormFieldModule } from '@angular/material/form-field';
import { BrowserModule } from '@angular/platform-browser';
import { MatIconModule } from '@angular/material/icon'
import { MatDialogModule } from '@angular/material/dialog';
import { MatCheckboxModule } from '@angular/material/checkbox';
import { MatSidenavModule } from '@angular/material/sidenav';
import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { HttpClientModule } from '@angular/common/http';
import { MissionPageComponent } from './pages/mission-page/mission-page.component';
import { RouterModule } from '@angular/router';
import { HomePageComponent } from './pages/home-page/home-page.component';
import { MissionHistoryComponent } from './pages/mission-history/mission-history.component';
import { RobotComponent } from './components/robot/robot.component';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { FormsModule } from '@angular/forms';
import { LogBoxComponent } from './components/log-box/log-box.component';
import { MapComponentComponent } from './components/map-component/map-component/map-component.component';
import { MissionHistoryDialog } from './components/mission-history-dialog/mission-history-dialog.component';
import { InitialLocationComponent } from './components/initial-location/initial-location.component';
import { MatInputModule } from '@angular/material/input';
import { MatButtonModule } from '@angular/material/button';
import { RobotEditorComponent } from './pages/robot-editor/robot-editor.component';
import { ErrorDialogComponent } from './components/error-dialog/error-dialog.component';
import { ReactiveFormsModule } from '@angular/forms';

@NgModule({
  declarations: [
    AppComponent,
    MissionPageComponent,
    HomePageComponent,
    MissionHistoryComponent,
    MissionHistoryDialog,
    RobotComponent,
    LogBoxComponent,
    MapComponentComponent,
    InitialLocationComponent,
    RobotEditorComponent,
    ErrorDialogComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    BrowserAnimationsModule,
    MatIconModule,
    MatDialogModule,
    MatSidenavModule,
    HttpClientModule,
    RouterModule,
    MatDialogModule,
    MatCheckboxModule,
    FormsModule,
    MatSlideToggleModule,
    MatFormFieldModule,
    MatInputModule,
    ReactiveFormsModule,
    MatButtonModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
