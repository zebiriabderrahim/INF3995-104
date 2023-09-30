import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { MissionPageComponent } from './pages/mission-page/mission-page.component';
import { HomePageComponent } from './pages/home-page/home-page.component';
import { MissionHistoryComponent } from './pages/mission-history/mission-history.component';

const routes: Routes = [
  { path: '', redirectTo: '/home', pathMatch: 'full' },
  { path: 'home', component: HomePageComponent },
  { path: 'mission', component: MissionPageComponent },
  { path: 'mission-history', component: MissionHistoryComponent },
  { path: '**', redirectTo: '/home' },
];

@NgModule({
  imports: [RouterModule.forRoot(routes, { useHash: true })],
  exports: [RouterModule]
})
export class AppRoutingModule {}




