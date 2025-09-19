const API_BASE_URL = 'http://192.168.1.66'; // !!! IMPORTANT: Replace with your Gateway's IP Address

// --- DOM Elements ---
const app = document.getElementById('app');
const navLinks = document.querySelectorAll('.nav-link');
const modalContainer = document.getElementById('modal-container');
const modalCloseBtn = document.getElementById('modal-close-btn');
const configForm = document.getElementById('config-form');
const modalTitle = document.getElementById('modal-title');

// --- State ---
let allDevices = [];

// --- API Functions ---
async function fetchData(endpoint) {
    try {
        const response = await fetch(`${API_BASE_URL}${endpoint}`);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        return await response.json();
    } catch (error) {
        console.error("Failed to fetch data:", error);
        app.innerHTML = `<p style="color:red;">Error: Could not connect to the gateway.</p>`;
        return null;
    }
}

async function postData(endpoint, data) {
    try {
        const response = await fetch(`${API_BASE_URL}${endpoint}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: new URLSearchParams(data).toString()
        });
        return await response.json();
    } catch (error) {
        console.error("Failed to post data:", error);
        alert('Error: Failed to send command to the gateway.');
        return null;
    }
}


// --- Modal Functions ---
function openModal(device) {
    modalTitle.textContent = `Configure: ${device.name}`;
    
    // Populate form with existing data (or defaults)
    configForm.elements.deviceName.value = device.name;
    // Note: The backend doesn't yet provide color/brightness data in the /api/devices response.
    // We would need to add it there to pre-fill correctly. For now, we use defaults.
    configForm.elements.on_color.value = device.config?.on_color || '#00ff00';
    configForm.elements.brightness_on.value = device.config?.brightness_on || 80;
    configForm.elements.off_color.value = device.config?.off_color || '#ff0000';
    configForm.elements.brightness_off.value = device.config?.brightness_off || 10;
    configForm.elements.linked_sensor_name.value = device.config?.linkedSensor || "";
    configForm.elements.daylight_threshold.value = device.config?.daylightThreshold || 0;

    // Populate sensor dropdown
    const sensorSelect = configForm.elements.linked_sensor_name;
    // Clear old options (except the first "-- None --")
    while (sensorSelect.options.length > 1) {
        sensorSelect.remove(1);
    }
    const envSensors = allDevices.filter(d => d.type === 'Env_Sensor');
    envSensors.forEach(sensor => {
        const option = document.createElement('option');
        option.value = sensor.name;
        option.textContent = sensor.name;
        sensorSelect.appendChild(option);
    });
    // Reselect the correct sensor for the device
    sensorSelect.value = device.config?.linkedSensor || "";

    modalContainer.classList.add('show');
}

function closeModal() {
    modalContainer.classList.remove('show');
}

// --- Page Render Functions ---
async function renderDevicesPage() {
    app.innerHTML = `<p>Loading devices...</p>`;
    const devices = await fetchData('/api/devices');
    if (!devices) return;
    
    allDevices = devices;

    let tableRows = devices.map(device => {
        let configureButton = (device.type === 'MultiSwitch') 
            ? `<button class="btn btn-configure" data-name="${device.name}">Configure</button>` 
            : '';
        const rediscoverButton = `<button class="btn btn-rediscover" data-name="${device.name}">Rediscover</button>`;
        return `
            <tr>
                <td>${device.name}</td>
                <td class="${device.isOnline ? 'status-online' : 'status-offline'}">${device.isOnline ? 'Online' : 'Offline'}</td>
                <td>${device.macAddress}</td>
                <td>${device.type}</td>
                <td>${configureButton} ${rediscoverButton}</td>
            </tr>`;
    }).join('');

    app.innerHTML = `
        <h2>Registered Devices</h2>
        <table>
            <thead>...</thead>
            <tbody>${tableRows}</tbody>
        </table>`;
}

async function renderStatusPage() {
    // ... (This function remains unchanged)
    app.innerHTML = `<p>Loading system status...</p>`;
    const status = await fetchData('/api/status');
    if (!status) return;

    app.innerHTML = `
        <h2>System Status</h2>
        <div class="status-grid">
            <strong>Uptime:</strong>         <span>${status.uptime}</span>
            <strong>Free Heap:</strong>      <span>${status.freeHeap} bytes</span>
            <strong>System Time:</strong>    <span>${status.systemTime}</span>
            <strong>IP Address:</strong>     <span>${status.network.ipAddress}</span>
            <strong>MQTT Connected:</strong> <span class="${status.mqtt.connected ? 'status-online' : 'status-offline'}">${status.mqtt.connected}</span>
            <strong>Device Count:</strong>   <span>${status.deviceCount}</span>
        </div>
        <h3 style="margin-top:2rem;">Watchdog Reset Log</h3>
        <div class="status-grid">
            <strong>Last Reset:</strong>     <span>${status.watchdog.log1}</span>
            <strong>Previous Reset:</strong> <span>${status.watchdog.log2}</span>
            <strong>Oldest Reset:</strong>   <span>${status.watchdog.log3}</span>
        </div>
    `;
}

function renderScenesPage() {
    app.innerHTML = `<h2>Scene Control</h2><p>Scene management UI is under construction.</p>`;
}

function renderActionsPage() {
    app.innerHTML = `<h2>Gateway Actions</h2><p>Action buttons are under construction.</p>`;
}

// --- Action Handler Function (Event Delegation) ---
function handleDeviceAction(event) {
    const target = event.target;
    const deviceName = target.dataset.name;
    if (!deviceName) return;

    if (target.classList.contains('btn-configure')) {
        const device = allDevices.find(d => d.name === deviceName);
        if (device) {
            openModal(device);
        }
    }
    if (target.classList.contains('btn-rediscover')) {
        if (confirm(`Are you sure you want to force rediscover "${deviceName}"?`)) {
            postData('/api/rediscover_device', { deviceName }).then(response => {
                if(response) alert(response.message);
            });
        }
    }
}


// --- Router ---
const routes = {
    'devices': renderDevicesPage,
    'status': renderStatusPage,
    'scenes': renderScenesPage,
    'actions': renderActionsPage,
};

function handleRouteChange() {
    const hash = window.location.hash.substring(1) || 'devices';
    const renderFunction = routes[hash];

    if (renderFunction) {
        navLinks.forEach(link => {
            link.classList.toggle('active', link.dataset.page === hash);
        });
        renderFunction();
    }
}

// --- Initial Setup ---
function init() {
    window.addEventListener('hashchange', handleRouteChange);
    handleRouteChange();
    
    app.addEventListener('click', handleDeviceAction);
    modalCloseBtn.addEventListener('click', closeModal);
    modalContainer.addEventListener('click', (event) => {
        if (event.target === modalContainer) {
            closeModal();
        }
    });
    
    configForm.addEventListener('submit', async (event) => {
        event.preventDefault();
        const formData = new FormData(configForm);
        const data = Object.fromEntries(formData.entries());
        
        const response = await postData('/api/save_device_config', data);
        if (response) {
            alert(response.message);
            closeModal();
        }
    });
}

init();
