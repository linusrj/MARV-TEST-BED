'strict mode';

import * as rclnodejs from 'rclnodejs';

async function main() {

  await rclnodejs.init();

  let node = rclnodejs.createNode('node_publisher', 'web_ui/gamepad');
  let publisher = node.createPublisher('std_msgs/msg/String', 'msg');
  
  rclnodejs.spin(node);

  setInterval(() => {
      const msg: rclnodejs.std_msgs.msg.String = {
	      data: 'cmdSteering'
      }
      console.log('publishing ', msg);
      publisher.publish(msg);
  }, 50);
}

main();