
document.addEventListener('keydown', function(e) {

  if (e.key === 'a' || e.key === 'A') {
    automate.checked = !automate.checked; // Toggle the checkbox
    automate.dispatchEvent(new Event('change')); // Trigger 'change' event manually
  }

  if (e.key === 'q' || e.key === 'Q') {
    scanQR.checked = !scanQR.checked; // Toggle the checkbox
    scanQR.dispatchEvent(new Event('change')); // Trigger 'change' event manually
  }

  const allowedKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
  if (allowedKeys.includes(e.key)) {

    if (isControllerActive === true){
      console.log("Switching to Arrows")
      isControllerActive = false;
      document.getElementById("controller").style.display="none";
      document.getElementById("arrow-grid").style.display="grid";
    }

    fetch('/key_control', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ key: e.key })
    })
    .then(response => response.json())
    .then(data => {
      console.log('Key sent successfully:', data);
    })
    .catch(error => {
      console.error('Error sending key:', error);
    });

    // Add active class to the corresponding arrow element
    const arrowId = 'arrow-' + e.key.replace('Arrow', '').toLowerCase();
    const arrowElement = document.getElementById(arrowId);
    if (arrowElement) {
      arrowElement.classList.add('active');
    }
  }
});

document.addEventListener('keyup', function(e) {
  const allowedKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
  if (allowedKeys.includes(e.key)) {
    // Remove the active class on key release
    const arrowId = 'arrow-' + e.key.replace('Arrow', '').toLowerCase();
    const arrowElement = document.getElementById(arrowId);
    if (arrowElement) {
      arrowElement.classList.remove('active');
    }
  }
});

// Send True or False for automating rover
const automate = document.getElementById('automate_toggle');
automate.addEventListener('change', function() {
  fetch('/automate_toggle', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ automate: this.checked })
  })
  .then(response => response.json())
  .then(data => {
    console.log('Key sent successfully:', data);
  })
  .catch(error => {
    console.error('Error sending key:', error);
  });
});

//True or False for scanning QR code in feed
const scanQR = document.getElementById('QR_toggle');
scanQR.addEventListener('change', function() {

  //Change stream
  if (this.checked){
    // document.getElementById('iframe').src = "http://" + raspi_ip + ":8889/QRDecode"
    // console.log("http://" + raspi_ip + ":8889/QRDecode")
    document.getElementById('iframe').classList.toggle('hidden');
    document.getElementById('iframe2').classList.toggle('hidden');


  }
  else {
    // document.getElementById('iframe').src = "http://" + raspi_ip + ":8889/LowLat"
    // console.log("http://" + raspi_ip + ":8889/LowLat")
    document.getElementById('iframe').classList.toggle('hidden');
    document.getElementById('iframe2').classList.toggle('hidden');
  }


});



// Listen for QR code updates via SSE and add them to the list
if (typeof(EventSource) !== "undefined") {
  //SSE request from client to server
  const source = new EventSource('/qr_feed');
  source.onmessage = function(event) {
    const qrList = document.getElementById("qr-list");
    // Remove placeholder if it exists
    const placeholder = document.getElementById("qr-placeholder");
    if (placeholder) {
      placeholder.remove();
    }
    // Create a new list item for the new QR code data
    const newItem = document.createElement("li");
    newItem.textContent = event.data;
    // Insert the new item at the top of the list
    qrList.insertBefore(newItem, qrList.firstChild);
  };
} else {
  console.error("Your browser doesn't support SSE.");
}

// Load controller SVG
fetch('/static/controller.svg')
.then(response => response.text())
.then(svg => {
  document.getElementById('controller-svg-container').innerHTML = svg;
});
