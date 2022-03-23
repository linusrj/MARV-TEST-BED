"use strict";

let valueString = ""
let oldvalueString = ""

const express = require("express");
const app = express();

let broadcaster;
const port = 4000;

const http = require("http");
const server = http.createServer(app);

const io = require("socket.io")(server);
app.use(express.static(__dirname + "/public"));

// Websocket
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

  // This is where Node.js server sees gamepad input, needs to be served
  socket.on("gamepad", (cmd) => {
    //console.log(cmd);
    valueString = cmd
  });
});
server.listen(port, () => console.log(`Server is running on port ${port}`));


// ROS node, publishes gamepad state to topic /web_ui/gamepad/msg
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs = require("rclnodejs");

async function main() {
    await rclnodejs.init();

    let node = rclnodejs.createNode('node_publisher', 'web_ui/gamepad');
    let publisher = node.createPublisher('std_msgs/msg/String', 'msg');
    rclnodejs.spin(node);

    setInterval(() => {
      // Only send NEW input to ROS
        if (valueString !== oldvalueString) {
          const msg = {
            data: valueString
          };
          console.log('Publishing: ', msg);
          publisher.publish(msg);
          oldvalueString = valueString;
        }
    }, 50);   // Send gamepad state every 50 ms
}

main();