// Tennis Ball Machine Controller JavaScript

const socket = io();
const monitor = document.getElementById('monitor');
const statusDiv = document.getElementById('status');
const homeBtn = document.getElementById('homeBtn');
const slider = document.getElementById('slider');
const needle = document.getElementById('needle');
const display = document.getElementById('display');
const gauge = document.querySelector('.gauge');
let messageCount = 0;
let currentMode = 'slider';
let isDragging = false;
let systemMode = 'manual';

function getMotorValue(motor) {
  if (currentMode === 'slider') {
    return document.getElementById(motor + 'Slider').value;
  } else {
    return document.getElementById(motor + 'Number').value;
  }
}

function updateValue(motor) {
  const value = getMotorValue(motor);
  document.getElementById(motor + 'Value').textContent = value;
  
  // Sync slider and number input
  if (currentMode === 'slider') {
    document.getElementById(motor + 'Number').value = value;
  } else {
    document.getElementById(motor + 'Slider').value = value;
  }
}

function switchMode() {
  const newMode = document.querySelector('input[name="systemMode"]:checked').value;
  
  // Send mode change to server
  socket.emit('set_mode', { mode: newMode });
  
  // Update UI immediately for responsiveness
  updateModeUI(newMode);
}

function updateModeUI(mode) {
  systemMode = mode;
  const currentModeText = document.getElementById('currentModeText');
  const autoTrackingStatus = document.getElementById('autoTrackingStatus');
  const stepperPanel = document.querySelector('.stepper-panel');
  
  if (mode === 'manual') {
    currentModeText.textContent = 'Manual Mode Active';
    currentModeText.style.color = '#00ff00';
    autoTrackingStatus.style.display = 'none';
    
    // Enable stepper panel
    stepperPanel.classList.remove('disabled');
    
  } else if (mode === 'auto') {
    currentModeText.textContent = 'Auto Mode Active';
    currentModeText.style.color = '#ff6600';
    autoTrackingStatus.style.display = 'block';
    
    // Disable entire stepper panel
    stepperPanel.classList.add('disabled');
  }
}


function toggleControlMode() {
  const mode = document.querySelector('input[name="controlMode"]:checked').value;
  currentMode = mode;
  
  const motors = ['pitch', 'topWheel', 'bottomWheel'];
  
  motors.forEach(motor => {
    const slider = document.getElementById(motor + 'Slider');
    const number = document.getElementById(motor + 'Number');
    
    if (mode === 'slider') {
      slider.style.display = 'block';
      number.style.display = 'none';
      // Sync values
      slider.value = number.value;
      updateValue(motor);
    } else {
      slider.style.display = 'none';
      number.style.display = 'block';
      // Sync values
      number.value = slider.value;
      updateValue(motor);
    }
  });
}

socket.on('status', function(data) {
  if (data.connected) {
    statusDiv.textContent = '● Arduino Connected';
    statusDiv.className = 'status connected';
    homeBtn.disabled = false;
  } else {
    statusDiv.textContent = '● Arduino Disconnected';
    statusDiv.className = 'status disconnected';
    homeBtn.disabled = true;
  }

  // Update mode if provided
  if (data.mode) {
    document.querySelector(`input[value="${data.mode}"]`).checked = true;
    updateModeUI(data.mode);
  }
});

socket.on('arduino_message', function(data) {
  addMessage(data.message, data.timestamp, 'arduino-text');
});

socket.on('message_sent', function(data) {
  addMessage('→ Sent: ' + data.message, data.timestamp, 'sent-text');
});

socket.on('error', function(data) {
  addMessage('Error: ' + data.message, new Date().toLocaleTimeString(), 'error-text');
});

socket.on('mode_changed', function(data) {
  // Update the radio button to match server state
  document.querySelector(`input[value="${data.mode}"]`).checked = true;
  
  // Update UI
  updateModeUI(data.mode);
  
  const message = `Mode changed to: ${data.mode}`;
  addMessage(message, data.timestamp, 'sent-text');
});

socket.on('tracking_update', function(data) {
  if (systemMode === 'auto') {
    const trackingInfo = document.getElementById('trackingInfo');
    
    if (data.person_detected) {
      trackingInfo.textContent = `Tracking person at (${data.x}, ${data.y}) - Stepper adjusting`;
      trackingInfo.style.color = '#00ff00';
    } else {
      trackingInfo.textContent = 'No person detected - Stepper idle';
      trackingInfo.style.color = '#ff6600';
    }
  }
});


function addMessage(message, timestamp, className) {
  if (messageCount === 0) monitor.innerHTML = '';
  messageCount++;
  const messageDiv = document.createElement('div');
  messageDiv.className = 'message';
  messageDiv.innerHTML = `<span class="timestamp">[${timestamp}]</span><span class="${className}">${message}</span>`;
  monitor.appendChild(messageDiv);
  monitor.scrollTop = monitor.scrollHeight;
}

function sendMotorCommand() {
  const pitch = getMotorValue('pitch');
  const top = getMotorValue('topWheel');
  const bottom = getMotorValue('bottomWheel');

  if (!pitch || !top || !bottom) {
    addMessage('Error: All fields must be filled', new Date().toLocaleTimeString(), 'error-text');
    return;
  }

  const version = "1.0";
  const msgId = Date.now();
  const coreMessage = `TOP_MOTOR=${top}|BOTTOM_MOTOR=${bottom}|LA=${pitch}`;
  const baseMessage = `MSG_START|${version}|${msgId}|${coreMessage}`;
  const checksum = baseMessage.split('').reduce((sum, char) => sum + char.charCodeAt(0), 0) % 256;
  const fullMessage = `${baseMessage}|#${checksum}|MSG_END`;

  socket.emit('send_message', { message: fullMessage });
}

function toggleStepperEnable() {
  const enabled = document.getElementById('enableToggle').checked;
  const command = enabled ? 'ENABLE' : 'DISABLE';
  
  // Enable/disable controls
  document.getElementById('homeBtn').disabled = !enabled;
  document.getElementById('slider').disabled = !enabled;
  document.getElementById('stepBtn').disabled = !enabled;
  
  // Visual feedback
  const gauge = document.querySelector('.gauge');
  const needle = document.getElementById('needle');
  if (enabled) {
    gauge.style.opacity = '1';
    needle.style.pointerEvents = 'auto';
  } else {
    gauge.style.opacity = '0.3';
    needle.style.pointerEvents = 'none';
  }
  
  const version = "1.0";
  const msgId = Date.now();
  const coreMessage = `STEPPER=${command}`;
  const baseMessage = `MSG_START|${version}|${msgId}|${coreMessage}`;
  const checksum = baseMessage.split('').reduce((sum, char) => sum + char.charCodeAt(0), 0) % 256;
  const fullMessage = `${baseMessage}|#${checksum}|MSG_END`;

  socket.emit('send_message', { message: fullMessage });
}

function homeStepper() {
  const version = "1.0";
  const msgId = Date.now();
  const coreMessage = `STEPPER=HOME`;
  const baseMessage = `MSG_START|${version}|${msgId}|${coreMessage}`;
  const checksum = baseMessage.split('').reduce((sum, char) => sum + char.charCodeAt(0), 0) % 256;
  const fullMessage = `${baseMessage}|#${checksum}|MSG_END`;

  socket.emit('send_message', { message: fullMessage });
}

function updateDirectionDisplay() {
  const isCCW = document.getElementById('dirToggle').checked;
  document.getElementById('dirDisplay').textContent = isCCW ? 'CCW' : 'CW';
}

function sendStepCommand() {
  const steps = document.getElementById('stepCount').value;
  const isCCW = document.getElementById('dirToggle').checked;
  const direction = isCCW ? 'CCW' : 'CW';
  
  if (!steps || steps <= 0) {
    addMessage('Error: Enter valid step count', new Date().toLocaleTimeString(), 'error-text');
    return;
  }
  
  const version = "1.0";
  const msgId = Date.now();
  const coreMessage = `STEPPER=STEP_${direction}_${steps}`;
  const baseMessage = `MSG_START|${version}|${msgId}|${coreMessage}`;
  const checksum = baseMessage.split('').reduce((sum, char) => sum + char.charCodeAt(0), 0) % 256;
  const fullMessage = `${baseMessage}|#${checksum}|MSG_END`;

  socket.emit('send_message', { message: fullMessage });
}

function updateAngle(angle) {
  // Clamp angle between 0 and 180
  angle = Math.max(0, Math.min(180, angle));
  
  const rotation = angle - 90;
  needle.style.transform = `translateX(-50%) rotate(${rotation}deg)`;
  display.textContent = Math.round(angle) + '°';
  slider.value = angle;
}

function getAngleFromMouse(e) {
  const rect = gauge.getBoundingClientRect();
  const centerX = rect.left + rect.width / 2;
  const centerY = rect.bottom;
  
  const mouseX = e.clientX || e.touches[0].clientX;
  const mouseY = e.clientY || e.touches[0].clientY;
  
  const deltaX = mouseX - centerX;
  const deltaY = centerY - mouseY;
  
  // Calculate angle in degrees
  let angle = Math.atan2(deltaX, deltaY) * (180 / Math.PI);
  
  // Convert to 0-180 range
  angle = angle + 90;
  
  return angle;
}

// Slider input
slider.addEventListener('input', function() {
  updateAngle(this.value);
});

// Needle dragging
needle.addEventListener('mousedown', function(e) {
  isDragging = true;
  needle.style.transition = 'none';
  e.preventDefault();
});

needle.addEventListener('touchstart', function(e) {
  isDragging = true;
  needle.style.transition = 'none';
  e.preventDefault();
});

document.addEventListener('mousemove', function(e) {
  if (isDragging) {
    const angle = getAngleFromMouse(e);
    updateAngle(angle);
  }
});

document.addEventListener('touchmove', function(e) {
  if (isDragging) {
    const angle = getAngleFromMouse(e);
    updateAngle(angle);
  }
});

document.addEventListener('mouseup', function() {
  if (isDragging) {
    isDragging = false;
    needle.style.transition = 'transform 0.1s ease';
  }
});

document.addEventListener('touchend', function() {
  if (isDragging) {
    isDragging = false;
    needle.style.transition = 'transform 0.1s ease';
  }
});

// Click on gauge to set angle
gauge.addEventListener('click', function(e) {
  if (e.target !== needle && !needle.contains(e.target)) {
    const angle = getAngleFromMouse(e);
    updateAngle(angle);
  }
});

function clearMonitor() {
  monitor.innerHTML = '<div style="color: #666; font-style: italic;">Monitor cleared. Arduino communication will appear here...</div>';
  messageCount = 0;
}

socket.on('connect', function() {
  addMessage('Connected to server', new Date().toLocaleTimeString(), 'sent-text');
});

socket.on('disconnect', function() {
  statusDiv.textContent = '● Server Disconnected';
  statusDiv.className = 'status disconnected';
  homeBtn.disabled = true;
  addMessage('Disconnected from server', new Date().toLocaleTimeString(), 'error-text');
});

// Initialize stepper controls as disabled
document.getElementById('homeBtn').disabled = true;
document.getElementById('slider').disabled = true;
document.querySelector('.gauge').style.opacity = '0.3';
document.getElementById('needle').style.pointerEvents = 'none';
