# WebRTC Video/Audio Broadcast

WebRTC PeerToPeer broadcast application that allows the broadcaster to send a video and audio stream to all connected users (watchers).


## Getting started

### Starting the application

Start the application using Node:

```bash
# Install dependencies for server
npm install

# Run the server
node server
```

### Testing the application

The application should now be running on your localhost:4000 and you test it by connecting to localhost:4000/broadcast.html to add a new broadcaster.

After that, you just need to visit localhost:4000 to connect to the server as a client and you should get the video that is streamed from the broadcaster.