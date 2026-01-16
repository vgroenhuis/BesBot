#pragma once

const char BESBOT_WIFI_HTML[] PROGMEM = R"MyString(
<!DOCTYPE html>
<html>
<head>
    <title>BesBot WiFi Manager</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 10px; margin: 20px 0; }
        .status-label { font-weight: bold; }
        .button { background: #007bff; color: white; border: none; padding: 10px 20px; margin: 5px; border-radius: 5px; cursor: pointer; }
        .button:hover { background: #0056b3; }
        .button:disabled { background: #ccc; cursor: not-allowed; }
    .drive-btn { min-width: 80px; min-height: 60px; font-size: 14px; user-select: none; -webkit-user-select: none; -ms-user-select: none; -moz-user-select: none; -webkit-touch-callout: none; -webkit-tap-highlight-color: rgba(0,0,0,0); }
        .drive-btn:active { background: #0056b3; transform: scale(0.95); }
        .pin-on { background: #28a745 !important; }
        .pin-off { background: #6c757d !important; }
        .networks { margin-top: 20px; }
        .network-item { background: #f8f9fa; border: 1px solid #dee2e6; margin: 5px 0; padding: 10px; border-radius: 5px; }
        .loading { color: #666; font-style: italic; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 BesBot Robot Manager</h1>
        <div>
            <a href="/wifi">Go to WiFi Manager</a>
        </div>
       
        <h2>🚗 Motor Control</h2>
        <div>
            <label>Mode: </label>
            <select id="motorMode" onchange="switchMotorMode()">
                <option value="pin_test">Output Pin Test</option>
                <option value="simple_drive">Simple Drive</option>
                <option value="joystick" selected>Joystick</option>
            </select>
        </div>
       
        <!-- Pin Test Mode -->
        <div id="pinTestMode" style="margin-top: 20px; display:none;">
            <h3>Pin Test Mode</h3>
            <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; max-width: 400px;">
                <button class="button" id="pin0" onclick="togglePin(0)">Pin 0 (GPIO 9): OFF</button>
                <button class="button" id="pin1" onclick="togglePin(1)">Pin 1 (GPIO 10): OFF</button>
                <button class="button" id="pin2" onclick="togglePin(2)">Pin 2 (GPIO 26): OFF</button>
                <button class="button" id="pin3" onclick="togglePin(3)">Pin 3 (GPIO 25): OFF</button>
            </div>
        </div>
        
        <!-- Simple Drive Mode -->
        <div id="simpleDriveMode" style="margin-top: 20px; display:none;">
            <h3>Simple Drive Mode</h3>
            <div style="display: grid; grid-template-columns: repeat(3, 100px); gap: 10px; justify-content: center; margin: 20px 0;">
                <div></div>
                <button class="button drive-btn" id="btnForward">↑ Forward</button>
                <div></div>
                <button class="button drive-btn" id="btnLeft">← Left</button>
                <div></div>
                <button class="button drive-btn" id="btnRight">→ Right</button>
                <div></div>
                <button class="button drive-btn" id="btnReverse">↓ Reverse</button>
                <div></div>
            </div>
        </div>

        <!-- Joystick Mode -->
        <div id="joystickMode" style="margin-top:20px;">
            <h3>Joystick Mode</h3>
            <p style="margin:6px 0; font-size:14px; color:#444;">Drag inside the circle. Center = stop. Up = forward. Left/right = turn. Diagonal mixes.</p>
            <div style="display:flex; justify-content:center;">
                <canvas id="joystickCanvas" width="260" height="260" style="touch-action:none; background:#fafafa; border:2px solid #aaa; border-radius:50%; transition:border-color 0.15s;"></canvas>
            </div>
            <div id="joystickStatus" style="text-align:center; margin-top:8px; font-size:13px; color:#555;">x: 0, y: 0</div>
            <div style="margin-top:10px; text-align:center; font-size:12px; color:#444;">
                <label>Update interval: <span id="joyIntervalLabel">100</span> ms
                    <input id="joyIntervalSlider" type="range" min="40" max="250" value="100" step="10" style="width:160px; vertical-align:middle; margin-left:8px;" />
                </label>
            </div>
        </div>
    </div>

    <script>
        // Motor control functions
        function switchMotorMode() {
            const mode = document.getElementById('motorMode').value;
            const pinTestMode = document.getElementById('pinTestMode');
            const simpleDriveMode = document.getElementById('simpleDriveMode');
            const joystickMode = document.getElementById('joystickMode');

            pinTestMode.style.display = 'none';
            simpleDriveMode.style.display = 'none';
            joystickMode.style.display = 'none';

            if (mode === 'pin_test') {
                pinTestMode.style.display = 'block';
                stopJoystick();
            } else if (mode === 'simple_drive') {
                simpleDriveMode.style.display = 'block';
                stopJoystick();
                drive('stop');
            } else if (mode === 'joystick') {
                joystickMode.style.display = 'block';
                drive('stop'); // ensure legacy drive stopped
                centerJoystick();
            }

            fetch('/motor/mode', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode })
            });
        }
        
        function togglePin(pinIndex) {
            fetch('/motor/pin', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ pin: pinIndex, action: 'toggle' })
            })
            .then(response => response.json())
            .then(data => {
                updatePinButtons();
            });
        }
        
        function drive(direction) {
            fetch('/motor/drive', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ direction: direction })
            });
        }
        
        function updatePinButtons() {
            fetch('/motor/status')
                .then(response => response.json())
                .then(data => {
                    data.pins.forEach(pin => {
                        const btn = document.getElementById('pin' + pin.index);
                        if (btn) {
                            const gpioNum = pin.gpio;
                            btn.textContent = `Pin ${pin.index} (GPIO ${gpioNum}): ${pin.state ? 'ON' : 'OFF'}`;
                            btn.className = pin.state ? 'button pin-on' : 'button pin-off';
                        }
                    });
                });
        }

        // Initialize drive buttons with passive touch listeners
        function initDriveButtons() {
            const mapping = [
                { id: 'btnForward', action: 'forward' },
                { id: 'btnLeft', action: 'left' },
                { id: 'btnRight', action: 'right' },
                { id: 'btnReverse', action: 'reverse' }
            ];
            mapping.forEach(cfg => {
                const el = document.getElementById(cfg.id);
                if (!el) return;
                // Prevent text selection / callout / context menu
                ['selectstart','contextmenu','dragstart'].forEach(ev => {
                    el.addEventListener(ev, e => { e.preventDefault(); e.stopPropagation(); });
                });
                // Mouse events
                el.addEventListener('mousedown', e => { e.preventDefault(); drive(cfg.action); });
                el.addEventListener('mouseup', () => drive('stop'));
                el.addEventListener('mouseleave', () => drive('stop'));
                // Touch events: use non-passive to allow preventDefault to suppress long-press selection
                el.addEventListener('touchstart', e => { try { e.preventDefault(); } catch(_) {}; drive(cfg.action); }, { passive: false });
                el.addEventListener('touchend', e => { try { e.preventDefault(); } catch(_) {}; drive('stop'); }, { passive: false });
                el.addEventListener('touchcancel', () => drive('stop'), { passive: true });
            });
        }

        // Joystick logic
        let lastJoySent = 0;
        let joyActive = false;
        let JOY_INTERVAL_MS = 100;
        let lastJoyDrawActiveMs = 0;
        const JOY_INACTIVE_MS = 600; // after this show inactive (red) border
        const JOY_IDLE_MS = 1200; // after this revert to neutral gray
        let lastJoyX = 0; let lastJoyY = 0; // track last command for keepalive
        const joyCanvas = () => document.getElementById('joystickCanvas');
        function joystickPost(x,y,force=false) {
            const now = Date.now();
            if (!force && (now - lastJoySent < JOY_INTERVAL_MS)) return; // throttle unless forced
            lastJoySent = now;
            fetch('/motor/joystick', {
                method:'POST', headers:{'Content-Type':'application/json'},
                body: JSON.stringify({ x, y })
            }).catch(()=>{});
        }
        function centerJoystick(send=true){
            const c = joyCanvas(); if(!c) return; const ctx = c.getContext('2d');
            drawJoystick(0,0); if(send) joystickPost(0,0,true); // force immediate stop packet
        }
        function stopJoystick(){ joyActive=false; centerJoystick(true); }
        function drawJoystick(nx,ny){
            const c = joyCanvas(); if(!c) return; const ctx = c.getContext('2d');
            const w=c.width, h=c.height; ctx.clearRect(0,0,w,h);
            const r=w/2 - 4; // outer radius
            ctx.beginPath(); ctx.arc(w/2,h/2,r,0,Math.PI*2); ctx.strokeStyle='#888'; ctx.lineWidth=2; ctx.stroke();
            // knob
            const knobR = 22; const x = w/2 + nx*r*0.7; const y = h/2 - ny*r*0.7; // invert y visually
            ctx.beginPath(); ctx.arc(x,y,knobR,0,Math.PI*2); ctx.fillStyle='#007bff'; ctx.fill(); ctx.strokeStyle='#004a9c'; ctx.stroke();
            document.getElementById('joystickStatus').textContent = `x: ${nx.toFixed(2)}, y: ${ny.toFixed(2)}`;
            lastJoyDrawActiveMs = Date.now();
            lastJoyX = nx; lastJoyY = ny;
        }

        // Visual inactivity indicator via border color
        setInterval(()=>{
            const c = joyCanvas(); if(!c) return; const age = Date.now() - lastJoyDrawActiveMs;
            if (joyActive) {
                c.style.borderColor = '#007bff'; // active blue
            } else if (age < JOY_INACTIVE_MS) {
                c.style.borderColor = '#ff9800'; // recently active (orange)
            } else if (age < JOY_IDLE_MS) {
                c.style.borderColor = '#d9534f'; // inactive warning (red)
            } else {
                c.style.borderColor = '#aaa'; // idle neutral
            }
        },150);

        // Slider to adjust throttle interval
        const slider = document.getElementById('joyIntervalSlider');
        const label = document.getElementById('joyIntervalLabel');
        slider.addEventListener('input', ()=>{
            JOY_INTERVAL_MS = parseInt(slider.value,10);
            label.textContent = JOY_INTERVAL_MS;
        });

        function handleJoyEvent(clientX, clientY){
            const c = joyCanvas(); if(!c) return; const rect=c.getBoundingClientRect();
            const cx = rect.left + rect.width/2; const cy = rect.top + rect.height/2;
            let dx = clientX - cx; let dy = clientY - cy; // dy positive down
            const maxR = rect.width/2;
            const mag = Math.sqrt(dx*dx + dy*dy);
            if (mag > maxR) { dx = dx*maxR/mag; dy = dy*maxR/mag; }
            // Normalize to -1..1, invert dy for logical y (up = forward)
            const nx = dx / maxR; const ny = -dy / maxR;
            drawJoystick(nx, ny);
            joystickPost(nx, ny);
        }

        function initJoystick(){
            const c = joyCanvas(); if(!c) return; centerJoystick(false);
            const start = (e)=>{ joyActive=true; const pt = (e.touches? e.touches[0]: e); handleJoyEvent(pt.clientX, pt.clientY); e.preventDefault(); };
            const moveLocal = (e)=>{ if(!joyActive) return; const pt = (e.touches? e.touches[0]: e); handleJoyEvent(pt.clientX, pt.clientY); e.preventDefault(); };
            const endLocal = (e)=>{ if(!joyActive) return; joyActive=false; centerJoystick(true); e.preventDefault(); };
            ['touchstart','mousedown'].forEach(ev=> c.addEventListener(ev,start,{passive:false}));
            ['touchmove','mousemove'].forEach(ev=> c.addEventListener(ev,moveLocal,{passive:false}));
            // End events (exclude mouseleave to avoid unintended stop)
            ['touchend','touchcancel','mouseup'].forEach(ev=> c.addEventListener(ev,endLocal,{passive:false}));
            // Global listeners: continue sending while outside canvas by clamping to edge
            window.addEventListener('mousemove', (e)=>{
                if(!joyActive) return;
                const rect = c.getBoundingClientRect();
                const cx = rect.left + rect.width/2; const cy = rect.top + rect.height/2;
                let dx = e.clientX - cx; let dy = e.clientY - cy;
                const maxR = rect.width/2;
                const mag = Math.sqrt(dx*dx + dy*dy);
                if (mag > maxR) { dx = dx*maxR/mag; dy = dy*maxR/mag; }
                const nx = dx / maxR; const ny = -dy / maxR;
                drawJoystick(nx, ny);
                joystickPost(nx, ny);
            }, {passive:false});
            window.addEventListener('mouseup', (e)=>{ if(!joyActive) return; joyActive=false; centerJoystick(true); }, {passive:false});
            // Remove blur auto-stop to avoid unintended zeroing during focus changes
        }

        // Keepalive resend every 1000ms ONLY if joystick not centered (reduce idle traffic)
        setInterval(()=>{
            if (Math.abs(lastJoyX) > 0.0001 || Math.abs(lastJoyY) > 0.0001) {
                joystickPost(lastJoyX, lastJoyY, true);
            }
        }, 1000);

    
        updatePinButtons();
        initDriveButtons();
        initJoystick(); // initialize joystick canvas & events
        // Ensure correct display mode based on default selection
        switchMotorMode();
        </script>
    </body>
</html>
)MyString";
