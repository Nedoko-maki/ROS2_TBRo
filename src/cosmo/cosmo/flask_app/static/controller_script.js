let controllerIndex = null;
let drift = 0.1
let counter = 0;
var isControllerActive = false

window.addEventListener("gamepadconnected", (event) => {
  handleConnect(event, true);
});

window.addEventListener("gamepaddisconnected", (event) => {
  handleConnect(event, false);
});

function handleConnect(event, connected) {
  const gamepad = event.gamepad;
  console.log(gamepad);
  
  if (connected) {
    controllerIndex = gamepad.index;
    // document.getElementById("controller").style.display="block";
    // document.getElementById("arrow-grid").style.display="none";
    
  } else {
    controllerIndex = null;
    // document.getElementById("controller").style.display="none";
    // document.getElementById("arrow-grid").style.display="grid";
  }
}

function updateControllerButton(index, value) {
  const button = document.getElementById(`controller-b${index}`);
  const selectedButtonClass = "selected-button";
  1;
  if (button) {
    if (value > 0) {
      button.classList.add(selectedButtonClass);
      button.style.filter = `contrast(${value * 200}%)`;
    } else {
      button.classList.remove(selectedButtonClass);
      button.style.filter = `contrast(100%)`;
    }
  }
}

function handleButtons(buttons) {
  for (let i = 0; i < buttons.length; i++) {
    const buttonValue = buttons[i].value;
    updateControllerButton(i, buttonValue);
  }
}

function handleSticks(axes) {
  updateStick("controller-b10", axes[0], axes[1]);
  updateStick("controller-b11", axes[2], axes[3]);
}

function updateStick(elementId, leftRightAxis, upDownAxis) {
  const multiplier = 25;
  const stickLeftRight = leftRightAxis * multiplier;
  const stickUpDown = upDownAxis * multiplier;

  const stick = document.getElementById(elementId);
  const x = Number(stick.dataset.originalXPosition);
  const y = Number(stick.dataset.originalYPosition);

  stick.setAttribute("cx", x + stickLeftRight);
  stick.setAttribute("cy", y + stickUpDown);
}

function gameLoop() {
  if (controllerIndex !== null) {
    const gamepad = navigator.getGamepads()[controllerIndex];

    let L2val = gamepad.buttons[6].value
    let R2val = gamepad.buttons[7].value
    let stickXaxis = Math.abs(gamepad.axes[2])
    
    // If R2, L2 or Right joystick detected, switch to controller on GUI 
    // and start sending directions back to pi
    if (L2val>0 || R2val>0 || stickXaxis>drift){

      
      if (isControllerActive === true && counter === 9) {
        //Send values to /controller_ controls endpoint
        //Every 10 cycles (reduce overhead)
        
        const controllerData = {
          R2: R2val,
          L2: L2val,
          rightStickX: stickXaxis
        };

        fetch('/controller_control', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(controllerData)
        }).catch(err => console.error('Failed to send controller data:', err));
      }

      if (isControllerActive === false){
        isControllerActive = true;
        document.getElementById("controller").style.display="block";
        document.getElementById("arrow-grid").style.display="none";
      }
      
      counter = (counter+1)%10;
      console.log(counter);
    }
    



    handleButtons(gamepad.buttons);
    handleSticks(gamepad.axes);
  }
  requestAnimationFrame(gameLoop);
}

gameLoop();
