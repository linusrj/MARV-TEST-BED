/*
    Node.js server for MARV Web Operator
    Establishes webRTC connection as well as handles websocket 
    communication between operator PC and REACH

    Only developed and tested with Node.js v16

    by Linus Johansson and Joakim Osterman, Spring 2022
*/

"use strict";

let valueString = ""
let oldvalueString = ""

const express = require("express");
const app = express();

let broadcaster;
const port = 1337;

const http = require("http");
const server = http.createServer(app);

const io = require("socket.io")(server);
app.use(express.static(__dirname + "/public"));

/* Websocket */
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

  // This is where Node.js server sees gamepad input
  socket.on("gamepad", (cmd) => {
    //console.log(cmd);
    valueString = cmd
  });
});
server.listen(port, () => console.log(`Server is running on port ${port}`));


/* ROS2 node */
Object.defineProperty(exports, "__esModule", { value: true });
const rclnodejs = require("rclnodejs");

async function main() {
    await rclnodejs.init();

    // Publishers
    let node = rclnodejs.createNode('web_operator', 'web_ui');
    let publisher = node.createPublisher('std_msgs/msg/String', 'gamepad');

    // This is where Node.js gets feedback from ROS and sends it to operator (websocket)
    node.createSubscription('std_msgs/msg/String', 'feedback', (msg) => {
      //console.log(msg)
      io.emit('feedback', msg)
    });

    rclnodejs.spin(node);

    setInterval(() => {
      // Only send NEW input to ROS
        if (valueString !== oldvalueString) {
          const msg = {
            data: valueString
          };
          //console.log('Publishing: ', msg);
          // ROS node, publishes gamepad state to topic /web_ui/gamepad
          publisher.publish(msg);
          oldvalueString = valueString;
        }
    }, 50);   // Send gamepad state every 50 ms
}

main();