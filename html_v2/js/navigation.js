document.addEventListener('DOMContentLoaded', function() {
    // Sidebar HTML Template
    const sidebarHTML = `
    <div id="sidebar" class="sidebar">
      <div class="sidebar-header">
        <button id="sidebar-close" class="btn-close-sidebar">&#9776;</button>
        <a href="/" class="brand">RADAR</a>
      </div>
      
      <nav>
        <a href="/" class="nav-link">Dashboard</a>
        
        <div class="nav-section">Display</div>
        <a href="/display/map/" class="nav-link">Delay-Doppler map</a>
        <a href="/display/maxhold/" class="nav-link">Max-hold delay Doppler map</a>
        <a href="/display/detection/delay/" class="nav-link">Detections in delay over time</a>
        <a href="/display/detection/doppler/" class="nav-link">Detections in Doppler over time</a>
        <a href="/display/detection/delay-doppler/" class="nav-link">Detections in delay-Doppler over time</a>
        <a href="/display/spectrum/" class="nav-link">Spectrum reference</a>
        <a href="/display/timing/" class="nav-link">Timing display</a>
        <a href="/display/calibration/" class="nav-link">Calibration & Sync Data</a>

        <div class="nav-section">API</div>
        <a href="/api/map" class="nav-link">Map data</a>
        <a href="/api/detection" class="nav-link">Detection data</a>
        <a href="/api/tracker" class="nav-link">Tracker data</a>
        <a href="/api/timing" class="nav-link">Timing data</a>
        <a href="/api/iqdata" class="nav-link">IQ metadata</a>
        <a href="/api/timestamp" class="nav-link">Latest timestamp</a>

        <div class="nav-section">Stash</div>
        <a href="/stash/map" class="nav-link">Map data</a>
        <a href="/stash/detection" class="nav-link">Detection data</a>
        <a href="/stash/iqdata" class="nav-link">IqData metadata</a>
        <a href="/stash/timing" class="nav-link">Timing data</a>
      </nav>

      <div class="status-panel">
        <button id="btn-capture" class="btn-control btn-success" onclick="toggleCapture()">
          Start Capture
        </button>
        <div class="mt-3 text-muted small">
          <div id="status-text">System Ready</div>
        </div>
      </div>
    </div>
    <button id="sidebar-toggle" class="btn-sidebar-toggle">&#9776;</button>
    `;

    // Inject Sidebar
    // Check if sidebar already exists (e.g. in index.html before we remove it)
    if (!document.getElementById('sidebar')) {
        document.body.insertAdjacentHTML('afterbegin', sidebarHTML);
    }

    // Highlight Active Link
    const currentPath = window.location.pathname;
    const navLinks = document.querySelectorAll('.sidebar .nav-link');
    navLinks.forEach(link => {
        // Handle trailing slashes for comparison
        const linkPath = new URL(link.href).pathname;
        if (linkPath === currentPath || (linkPath !== '/' && currentPath.startsWith(linkPath))) {
            link.classList.add('active');
        }
    });

    // API Port Rewriting
    const host = window.location.hostname;
    const isLocalHost = (host === 'localhost' || host === '127.0.0.1' || host.startsWith('192.168.') || host.startsWith('10.') || host.startsWith('172.'));
    
    if (isLocalHost) {
        const anchors = document.getElementsByTagName("a");
        for (var i = 0; i < anchors.length; i++) {
            if (anchors[i].href.includes('/api/') || anchors[i].href.includes('/stash/')) {
                try {
                    var url = new URL(anchors[i].href);
                    url.port = "3000";
                    anchors[i].href = url.toString();
                } catch (e) {
                    console.error("Error rewriting URL:", anchors[i].href, e);
                }
            }
        }
    }

    // Sidebar Toggle Logic
    const sidebar = document.getElementById('sidebar');
    const toggleBtn = document.getElementById('sidebar-toggle');
    const closeBtn = document.getElementById('sidebar-close');
    const body = document.body;

    function toggleSidebar() {
        sidebar.classList.toggle('collapsed');
        body.classList.toggle('sidebar-collapsed');
        
        // Save state
        const isCollapsed = sidebar.classList.contains('collapsed');
        localStorage.setItem('sidebarCollapsed', isCollapsed);
    }

    if (toggleBtn) {
        toggleBtn.addEventListener('click', toggleSidebar);
    }
    if (closeBtn) {
        closeBtn.addEventListener('click', toggleSidebar);
    }

    // Restore state
    const savedState = localStorage.getItem('sidebarCollapsed');
    if (savedState === 'true') {
        sidebar.classList.add('collapsed');
        body.classList.add('sidebar-collapsed');
    }
});
