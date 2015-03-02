var host = document.getElementById('host').value;
var socket = new WebSocket("ws://" + host + ":8888/mazebuster");

// When the connection is open, send some data to the server
socket.onopen = function () {
  // connection.send('Ping'); // Send the message 'Ping' to the server
};

// Log errors
socket.onerror = function (error) {
  console.log('WebSocket Error ' + error);
};

// Log messages from the server
socket.onmessage = function (e) {
  console.log('Server: ' + e.data);
};