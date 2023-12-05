import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { environment } from 'src/environments/environment';

@Injectable({
    providedIn: 'root',
})
// code used from professor Nikolay Radoev's gitlab: https://gitlab.com/nikolayradoev/socket-io-exemple
export class ClientSocketService {
  socket!: Socket;

  isSocketAlive() {
    /***
     * Check if the socket is alive
     * @return boolean: true if the socket is alive
     */
    return this.socket && this.socket.connected;
  }

  connect() {
    /***
     * Connect to the socket
     */
    this.socket = io(environment.serverURL, { transports: ['websocket'], upgrade: false });
  }

  disconnect() {
    /***
     * Disconnect from the socket
     */
    this.socket.disconnect();
  }

  on<T>(event: string, action: (data: T) => void): void {
    /***
     * Listen to an event
     * @param event: event to listen to
     * @param action: action to do when the event is triggered
     */
    this.socket.on(event, action);
  }

  send<T>(event: string, data?: T): void {
    /***
     * Send an event
     * @param event: event to send
     * @param data: data to send with the event
     */
    if (data) {
      this.socket.emit(event, data);
    } else {
      this.socket.emit(event);
    }
  }
}
