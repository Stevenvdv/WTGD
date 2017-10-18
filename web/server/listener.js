

// Require rosnodejs itself
const rosnodejs = require('rosnodejs');

// Requires the std_msgs message package
const std_msgs = rosnodejs.require('std_msgs').msg;

function listener() {
  // Register node with ROS master
  rosnodejs.initNode('/listener_node')
    .then((rosNode) => {
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      let sub = rosNode.subscribe('/chatter', std_msgs.String,
        (data) => { // define callback execution
          rosnodejs.log.info('I heard: [' + data.data + ']');
        }
      );
    });
}

listener();