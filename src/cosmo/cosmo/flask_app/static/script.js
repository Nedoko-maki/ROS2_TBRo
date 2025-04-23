document.addEventListener('keydown', function(e) {
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
