import { NgModule } from '@angular/core';
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
    MatSlideToggleModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
