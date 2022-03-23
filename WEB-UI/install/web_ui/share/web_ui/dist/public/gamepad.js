let enabled = false;

window.addEventListener("gamepadconnected", event => {
    console.log("Gamepad connected:")
    console.log(event.gamepad)
})

window.addEventListener("gamepaddisconnected", event => {
    console.log("Gamepad disconnected:")
    console.log(event.gamepad)
})

const enableGamepadButton = document.querySelector("#enable-gamepad");
enableGamepadButton.addEventListener("click", enableGamepad)

function enableGamepad() {
    enabled = !enabled;

    if (enabled) {
        document.getElementById('enable-gamepad').innerHTML = "Disable Gamepad";
        console.log("Gamepad input enabled");
    } else {
        document.getElementById('enable-gamepad').innerHTML = "Enable Gamepad";
        console.log("Gamepad input disabled");
    }
}

function update() {
    const gamepads = navigator.getGamepads()
    if (gamepads[0] && enabled) {
        // Convert axes to valid range before emitting over socket
        
        socket.emit('gamepad', JSON.stringify({ speed: gamepads[0].buttons[7].value.toFixed(2),
                                                angle: gamepads[0].axes[0].toFixed(2), }), null, 2);
    }
    window.requestAnimationFrame(update)
}

window.requestAnimationFrame(update)