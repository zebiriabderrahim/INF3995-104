import { TestBed } from '@angular/core/testing';
import { RouterTestingModule } from '@angular/router/testing';
import { AppComponent } from './app.component';
import { SocketService } from './services/socket-service/socket.service';

describe('AppComponent', () => {
  const mockSocketService = {
    connect: () => {},
    handleSocket: () => {},  
  };
  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [
        RouterTestingModule
      ],
      declarations: [
        AppComponent
      ],
      providers: [
        { provide: SocketService, useValue: mockSocketService },
      ],
    }).compileComponents();
  });

  it('should create the app', () => {
    const firstSpy= spyOn(mockSocketService,'connect');
    const secondSpy= spyOn(mockSocketService,'handleSocket');
    const fixture = TestBed.createComponent(AppComponent);
    const app = fixture.componentInstance;
    expect(app).toBeTruthy();
    expect(firstSpy).toHaveBeenCalled();
    expect(secondSpy).toHaveBeenCalled();
  });

  it(`should have as title 'front-end'`, () => {
    const fixture = TestBed.createComponent(AppComponent);
    const app = fixture.componentInstance;
    expect(app.title).toEqual('front-end');
  });

  // it('should render title', () => {
  //   const fixture = TestBed.createComponent(AppComponent);
  //   fixture.detectChanges();
  //   const compiled = fixture.nativeElement as HTMLElement;
  // });
});
