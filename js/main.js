var robotCounter = 1;
var cartCounter = 1;
let robotData = {};
let cartData = {};

const http = require('http');
const fs = require('fs');
const WebSocket = require('ws'); // Import WebSocket

const server = http.createServer((req, res) => {
    // Handle HTTP requests
    // ...
});

const wss = new WebSocket.Server({ server }); // Create a WebSocket server

// WebSocket connection handling
wss.on('connection', (ws) => {
    // Send the initial dictionary data when a client connects
    ws.send(JSON.stringify(robotData));

    // Listen for updates from the client
    ws.on('message', (data) => {
        // Update the dictionary variable
        robotData = JSON.parse(data);

        // Broadcast the updated dictionary to all connected clients
        wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify(robotData));
            }
        });
    });
});

server.listen(8000, () => {
    console.log('Server running on port 8000');
});


function addRobot() {
    // Create a new row for the robot table
    var newRow = document.getElementById("robotTable").insertRow();
    var nameCell = newRow.insertCell(0);
    var var1Cell = newRow.insertCell(1);
    var var2Cell = newRow.insertCell(2);
    var var3Cell = newRow.insertCell(3);
    var var4Cell = newRow.insertCell(4);
    var var5Cell = newRow.insertCell(5);
    var var6Cell = newRow.insertCell(6);
    var var7Cell = newRow.insertCell(7);
    var deleteCell = newRow.insertCell(8);

    nameCell.innerHTML = 'Chargepal_' + String(robotCounter);
    var1Cell.innerHTML = 'unknown';
    var2Cell.innerHTML = 'unknown';
    var3Cell.innerHTML = 'unknown';
    var4Cell.innerHTML = 'unknown';
    var5Cell.innerHTML = 'unknown';
    var6Cell.innerHTML = 'False';
    var7Cell.innerHTML = '100.0';

    deleteCell.innerHTML = '<button onclick="deleteRobotRow(this)">Delete</button>';
       
    // Increment the robot counter for the next addition
    robotCounter++;

    // Storing the values in a dictonary 
    robot_info = []
    for (var i = 1; i < newRow.cells.length - 1; i++) {
        robot_info.push(newRow.cells[i].textContent);
    }
    robotData[newRow.cells[0].textContent] = robot_info;

    
}

function addCart() {
    // Create a new row for the cart table
    var newRow = document.getElementById("cartTable").insertRow();
    var nameCell = newRow.insertCell(0);
   
    var var1Cell = newRow.insertCell(1);
    var select_cart_position = document.createElement("select");
    var options_cart_position = ["BWS", "BCS"];

    // Populate the select element with options
    for (var i = 0; i < options_cart_position.length; i++) {
        var option_cart_position = document.createElement("option");
        option_cart_position.value = String(options_cart_position[i]);
        option_cart_position.text = String(options_cart_position[i]);
        select_cart_position.appendChild(option_cart_position);
    }
    var1Cell.appendChild(select_cart_position);

    var var2Cell = newRow.insertCell(2);
    var select_plug_status = document.createElement("select");
    var options_plug_status = ["True", "False"];
    // Populate the select element with options
    for (var j = 0; j < options_plug_status.length; j++) {
        var option_plug_status = document.createElement("option");
        option_plug_status.value = String(options_plug_status[j]);
        option_plug_status.text = String(options_plug_status[j]);
        select_plug_status.appendChild(option_plug_status);
        }

    var var3Cell = newRow.insertCell(3);
    var deleteCell = newRow.insertCell(4);

    // Customize the row content here, e.g., add logos, names, variables, etc.
    nameCell.innerHTML = 'BAT_' + String(cartCounter);
    var1Cell.innerHTML = select_cart_position.outerHTML;
    var2Cell.innerHTML = select_plug_status.outerHTML;
    var3Cell.innerHTML = 'unknown';

    deleteCell.innerHTML = '<button onclick="deleteCartRow(this)">Delete</button>';
    cartCounter ++;

    cart_info = []
    for (var i = 1; i < newRow.cells.length - 1; i++) {
        cart_info.push(newRow.cells[i].textContent);
    }
    cartData[newRow.cells[0].textContent] = cart_info;
}   

function deleteRobotRow(button) {
    var row = button.parentNode.parentNode;
    row.parentNode.removeChild(row);
    delete robotData[row.cells[0].textContent]
    robotCounter--;
}

function deleteCartRow(button) {
    var row = button.parentNode.parentNode;
    row.parentNode.removeChild(row);
    delete cartData[row.cells[0].textContent]
    cartCounter--;
}
