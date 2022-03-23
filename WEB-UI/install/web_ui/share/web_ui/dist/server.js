"use strict";
// Example-1, Create Node and simple Publisher
'strict mode';

const express = require("express");
const app = express();

let broadcaster;
const port = 4000;

const http = require("http");
const server = http.createServer(app);

const io = require("socket.io")(server);
app.use(express.static(__dirname + "/public"));

io.sockets.on("error", e => console.log(e));
io.sockets.on("connection", socket => {
  socket.on("broadcaster", () => {
    broadcaster = socket.id;
    socket.broadcast.emit("broadcaster");
  });
  socket.on("watcher", () => {
    socket.to(broadcaster).emit("watcher", socket.id);
  });
  socket.on("offer", (id, message) => {
    socket.to(id).emit("offer", socket.id, message);
  });
  socket.on("answer", (id, message) => {
    socket.to(id).emit("answer", socket.id, message);
  });
  socket.on("candidate", (id, message) => {
    socket.to(id).emit("candidate", socket.id, message);
  });
  socket.on("disconnect", () => {
    socket.to(broadcaster).emit("disconnectPeer", socket.id);
  });

  // This is where server sees gamepad input, needs to be served
  socket.on("gamepad", (cmd) => {
    console.log(cmd);
  });
});
server.listen(port, () => console.log(`Server is running on port ${port}`));


// Example-1, Create Node and simple Publisher
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs = require("rclnodejs");
async function main() {
    // initialize the ros-client-library
    await rclnodejs.init();
    // create generic message publisher and begin sending message every second
    let node = rclnodejs.createNode('node_publisher', 'ros2_js_examples');
    let publisher = node.createPublisher('std_msgs/msg/String', 'msg');
    rclnodejs.spin(node);
    // publish helloworld String msg every second
    setInterval(() => {
        const msg = {
            data: 'hello ROS2 from rclnodejs'
        };
        console.log('publishing ', msg);
        publisher.publish(msg);
    }, 1000);
}
// run the program
main();
//# sourceMappingURL=server.js.map
