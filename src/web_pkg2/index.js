var manager;
var topics;
var mcanvas ;
var ctx ;
var PI2 = Math.PI * 2;
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});
var new_listener;
var array_listeners = [];
var messageMap_suscribed= new Map()


array_listeners.push( new ROSLIB.Topic({
  ros : ros,
  name : '/turtle1/pose',
  messageType : 'turtlesim/Pose',
  info: false
}))

array_listeners[array_listeners.length-1].subscribe(function(m) { 
    if(m){
      document.getElementById("x").innerHTML = String(m.x).substring(0, 4);
      document.getElementById("y").innerHTML = String(m.y).substring(0, 4);
      if(ctx){
        ctx.beginPath();
        ctx.arc((m.x)*10,(m.y)*10, 3, 0, PI2);
        ctx.closePath();
        ctx.fill();
      }
    }
  //move(1, 0);
});


ros.getParams(function(params) {
//     console.error("nodes",params);
    });
    
ros.getMessageDetails('turtlesim/Pose',function(params) {
//  console.error("getMessageDetails",params);
 });

cmd_vel_listener = new ROSLIB.Topic({
  ros : ros,
  name : "/turtle1/cmd_vel",
  messageType : 'geometry_msgs/Twist'
});

move = function (linear, angular) {
  var twist = new ROSLIB.Message({
    linear: {
      x: linear,
      y: 0,
      z: 0
    },
    angular: {
      x: 0,
      y: 0,
      z: angular
    }
  });
  cmd_vel_listener.publish(twist);
}

createJoystick = function () {
  var options = {
    zone: document.getElementById('zone_joystick'),
    threshold: 0.1,
    position: { left: 40 + '%' },
    mode: 'static',
    size: 100,
    color: '#000000',
  };
    // nipple.js for the jostick that will be used
    manager = nipplejs.create(options);

    linear_speed = 0;
    angular_speed = 0;

    self.manager.on('start', function (event, nipple) {
      //console.log("Movement start");
    });

    self.manager.on('move', function (event, nipple) {
      //console.log("Moving");
    });

    self.manager.on('end', function () {
      //console.log("Movement end");
    });
  }
  window.onload = function () {
    createJoystick();

    manager.on('start', function (event, nipple) {
      timer = setInterval(function () {
          move(linear_speed, angular_speed);
      }, 25);
      });

      manager.on('end', function () {
      if (timer) {
          clearInterval(timer);
      }
      self.move(0, 0);
      });

      manager.on('move', function (event, nipple) {
      max_linear = 5.0; // m/s
      max_angular = 2.0; // rad/s
      max_distance = 75.0; // pixels;
      linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
      angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
      });
      mcanvas = document.getElementById("canvas1");
      ctx = mcanvas.getContext("2d");
      ctx.strokeStyle = "blue";
}

