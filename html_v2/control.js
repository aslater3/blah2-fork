var host = window.location.hostname;

function toggleCapture() {
  var url = 'http://' + host + ':3000/capture/toggle';
  
  // Simple UI feedback
  var btn = document.getElementById('btn-capture');
  var originalText = btn ? btn.innerText : '';
  
  if (btn) {
    btn.disabled = true;
    btn.style.opacity = '0.7';
  }

  $.getJSON(url, function () { })
    .done(function (data) {
      console.log('API worked');
      if (btn) {
        // Toggle button state
        if (btn.classList.contains('btn-success')) {
            btn.classList.remove('btn-success');
            btn.classList.add('btn-danger');
            btn.innerText = 'Stop Capture';
        } else {
            btn.classList.remove('btn-danger');
            btn.classList.add('btn-success');
            btn.innerText = 'Start Capture';
        }
      }
    })
    .fail(function () {
      console.log('API Fail');
      if (btn) {
        // Flash error
        var currentClass = btn.classList.contains('btn-success') ? 'btn-success' : 'btn-danger';
        btn.classList.remove(currentClass);
        btn.classList.add('btn-warning'); // Assuming bootstrap warning color
        btn.innerText = 'Error';
        
        setTimeout(() => {
            btn.classList.remove('btn-warning');
            btn.classList.add(currentClass);
            btn.innerText = originalText;
        }, 1000);
      }
    })
    .always(function () {
      if (btn) {
        btn.disabled = false;
        btn.style.opacity = '1';
      }
    });
}

$(document).on('keypress', function (e) {
  if (e.which == 32) { // Spacebar
    toggleCapture();
  }
});
