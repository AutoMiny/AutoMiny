var connection = new WebSocket("ws://" + location.host);
connection.binaryType = 'arraybuffer';

/* Server Input */
function onMessageCallback(event) {
  var speed = new Int16Array(event.data, 0, 1)[0];
  var steering = new Int16Array(event.data, 2, 1)[0];

  var imageBlob = new Blob([new DataView(event.data, 4)], {type: 'application/octet-binary'});
  var imageBlobUrl = URL.createObjectURL(imageBlob);

  var canvas = document.getElementById('image-canvas');
  var context = canvas.getContext('2d');
  var image = new Image();
  image.onload = function() {
    URL.revokeObjectURL(imageBlobUrl);
    context.drawImage(image, 0, 0, window.innerWidth, window.innerHeight);
  };
  image.src = imageBlobUrl;

  document.getElementById("speed").innerHTML = "SPEED: " + speed;
  document.getElementById("steering").innerHTML = "STEERING: " + steering; 
}
connection.onmessage = onMessageCallback;


/* Joystick */
var speedJoystickEnabled = false;
var steeringJoystickEnabled = false;
var joystickSpeed = 0.5;
var joystickSteering = 0.5;

var speedJoystickRect;
var steeringJoystickRect;
var joystickRadius = 0.3;

function initializeJoysticks() {
  var steeringCanvas = document.getElementById("steering-canvas");
  var speedCanvas = document.getElementById("speed-canvas");

  /* Mouse Events */
  steeringCanvas.addEventListener('mousedown', (event) => {
    enableSteeringJoystick(event.button == 0);
    moveSteeringJoystick(event.clientX, event.clientY);
    drawSteeringJoystick(steeringCanvas);
  });
  steeringCanvas.addEventListener('mousemove', (event) => {
    moveSteeringJoystickConditional(event.clientX, event.clientY);
    drawSteeringJoystick(steeringCanvas);
  });
  steeringCanvas.addEventListener('mouseup', (event) => {
    disableSteeringJoystick(event);
    drawSteeringJoystick(steeringCanvas);
  });
  steeringCanvas.addEventListener('mouseleave', (event) => {
    disableSteeringJoystick();
    drawSteeringJoystick(steeringCanvas);
  });

  speedCanvas.addEventListener('mousedown', (event) => {
    enableSpeedJoystick(event.button == 0);
    moveSpeedJoystick(event.clientX, event.clientY);
    drawSpeedJoystick(speedCanvas);
  });
  speedCanvas.addEventListener('mousemove', (event) => {
    moveSpeedJoystickConditional(event.clientX, event.clientY);
    drawSpeedJoystick(speedCanvas);
  });
  speedCanvas.addEventListener('mouseup', (event) => {
    disableSpeedJoystick(event);
    drawSpeedJoystick(speedCanvas);
  });
  speedCanvas.addEventListener('mouseleave', (event) => {
    disableSpeedJoystick(event);
    drawSpeedJoystick(speedCanvas);
  });
  
  /* Touch Events */
  document.addEventListener('touchstart', (event) => {
    for(var i = 0; i < event.touches.length; i++) {
      var touch = event.touches.item(i);
      if (touch.target.id == "steering-canvas") {
        enableSteeringJoystick();
        moveSteeringJoystick(touch.clientX, touch.clientY);
        drawSteeringJoystick(steeringCanvas);
      }
      else if (touch.target.id == "speed-canvas") {
        enableSpeedJoystick();
        moveSpeedJoystick(touch.clientX, touch.clientY);
        drawSpeedJoystick(speedCanvas);
      }
    }
  });
  document.addEventListener('touchmove', (event) => {
    for(var i = 0; i < event.touches.length; i++) {
      var touch = event.touches.item(i);
      if (touch.target.id == "steering-canvas") {
        moveSteeringJoystickConditional(touch.clientX, touch.clientY);
        drawSteeringJoystick(steeringCanvas);
      }
      else if (touch.target.id == "speed-canvas") {
        moveSpeedJoystickConditional(touch.clientX, touch.clientY);
        drawSpeedJoystick(speedCanvas);
      }
    }
  });
  document.addEventListener('touchend', (event) => {
    var speedJoystickTouched = false;
    var steeringJoystickTouched = false;

    for(var i = 0; i < event.touches.length; i++) {
      var touch = event.touches.item(i);
      if (touch.target.id == "steering-canvas") {
        speedJoystickTouched = true;
      }
      else if (touch.target.id == "speed-canvas") {
        steeringJoystickTouched = true;
      }
    }

    if(!speedJoystickTouched) {
      disableSpeedJoystick(event);
      drawSpeedJoystick(speedCanvas);
    }
    if(!steeringJoystickTouched) {
      disableSteeringJoystick(event);
      drawSteeringJoystick(steeringCanvas);
    }
  });
}

function enableSpeedJoystick(enable = true) {
  speedJoystickEnabled = enable;
}

function enableSteeringJoystick(enable = true) {
  steeringJoystickEnabled = enable;
}

function moveSpeedJoystickConditional(x, y) {
  if (!speedJoystickEnabled) return;
  moveSpeedJoystick(x, y);
}

function moveSteeringJoystickConditional(x, y) {
  if (!steeringJoystickEnabled) return;
  moveSteeringJoystick(x, y);
}

function moveSpeedJoystick(x, y) {
  joystickSpeed = y - (speedJoystickRect.top + speedJoystickRect.height * (1.0 - joystickRadius * 2.0) * 0.5);
  joystickSpeed = 1.0 - joystickSpeed  / (speedJoystickRect.height * joystickRadius * 2.0);
  joystickSpeed = 1.0 - Math.min(Math.max(0.0, joystickSpeed), 1.0);
  connection.send([joystickSpeed, joystickSteering]);
}

function moveSteeringJoystick(x, y) {
  joystickSteering = x - (steeringJoystickRect.left + steeringJoystickRect.width * (1.0 - joystickRadius * 2.0) * 0.5);
  joystickSteering = joystickSteering  / (steeringJoystickRect.width * joystickRadius * 2.0);
  joystickSteering = 1.0 - Math.min(Math.max(0.0, joystickSteering), 1.0);
  connection.send([joystickSpeed, joystickSteering]);
}

function disableSpeedJoystick(event) {
  speedJoystickEnabled = false;
  joystickSpeed = 0.5;
  connection.send([joystickSpeed, joystickSteering]);
}

function disableSteeringJoystick(event) {
  steeringJoystickEnabled = false;
  joystickSteering = 0.5;
  connection.send([joystickSpeed, joystickSteering]);
}


/* Window */
function onResize() {
  document.getElementById("container").style.height = window.innerHeight + "px";
  document.getElementById("container").style.width = window.innerWidth + "px";

  document.getElementById("image-canvas").width = window.innerWidth;
  document.getElementById("image-canvas").height = window.innerHeight;

  var joystickSize = Math.max(window.innerWidth, window.innerHeight) * 0.25;
  var steeringCanvas = document.getElementById("steering-canvas");
  var speedCanvas = document.getElementById("speed-canvas");
  steeringCanvas.width = steeringCanvas.height = joystickSize;
  speedCanvas.width = speedCanvas.height = joystickSize;
  document.getElementById("speed").style.width = joystickSize + "px";
  document.getElementById("steering").style.width = joystickSize + "px";

  speedJoystickRect = speedCanvas.getBoundingClientRect();
  steeringJoystickRect = steeringCanvas.getBoundingClientRect();

  drawSteeringJoystick(steeringCanvas);
  drawSpeedJoystick(speedCanvas);
}
window.onresize = onResize;

window.onload = function() {
  onResize();
  initializeJoysticks();
}


/* Drawing */
function drawJoystick(canvas, xFactor, yFactor) {
  var context = canvas.getContext('2d');

  context.clearRect(0, 0, canvas.width, canvas.height);
  drawBackground(canvas, context);
  drawKnob(canvas, context, xFactor, yFactor);
}

function drawBackground(canvas, context) {
  context.beginPath();
  context.arc(canvas.width * 0.5, canvas.height * 0.5, canvas.width * 0.3, 0, 2 * Math.PI, false);
  context.fillStyle = '#333333';
  context.fill();
  context.closePath();
}

function drawKnob(canvas, context, xFactor, yFactor) {
  context.beginPath();
  context.arc(canvas.width * xFactor, canvas.height * (1.0 - yFactor), canvas.width * 0.15, 0, 2 * Math.PI, false);
  context.fillStyle = '#666666';
  context.fill();
  context.closePath();
}

function drawSpeedJoystick(canvas) {
  drawSpeed = (1.0 - joystickSpeed) / (canvas.height / (canvas.height * joystickRadius * 2.0 + (1.0 - joystickRadius * 2.0) * 0.5)) + (1.0 - joystickRadius * 2.0) * 0.5;
  drawJoystick(canvas, 0.5, drawSpeed);
}

function drawSteeringJoystick(canvas) {
  drawSteering = (1.0 - joystickSteering) / (canvas.width / (canvas.width * joystickRadius * 2.0 + (1.0 - joystickRadius * 2.0) * 0.5)) + (1.0 - joystickRadius * 2.0) * 0.5;
  drawJoystick(canvas, drawSteering, 0.5);
}
