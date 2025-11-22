# Implementation Plan - Web UI Modernization & Docker Optimization

[Overview]
Modernize the passive radar web interface with a "Clean Dark" SaaS aesthetic and optimize the Docker deployment.
The goal is to transform the current basic Bootstrap UI into a professional-looking dashboard with a dark theme, improved typography, and better UX for controls.
**Crucially, the new interface will be deployed alongside the old one.** We will create a new directory `html_v2` for the modernized UI and serve it via a new Nginx container on port 49153, while keeping the original Apache container on port 49152 serving the original `html` directory.

[Types]
No changes to the C++ backend types or API data structures are required. The changes are purely frontend (CSS/HTML/JS) and infrastructure (Docker).

[Files]
- **New Directory:** `html_v2/` (Copied from `html/` as a base)
- **Modified:** `html_v2/lib/blah2.css`
    - Replace existing styles with a new dark theme using CSS variables.
    - Implement modern typography (Inter/Roboto Mono).
    - Add styles for dashboard layout, cards, and controls.
- **Modified:** `html_v2/index.html`
    - Restructure layout to a full-screen dashboard.
    - Add a sidebar/header for navigation and status.
    - Add visible "Start/Stop Capture" button.
- **Modified:** `html_v2/js/plot_map.js`
    - Update Plotly configuration for dark mode (background color, grid lines, font colors).
    - Adjust color scales to match the new theme.
- **Modified:** `html_v2/control.js`
    - Add event listeners for the new UI buttons (replacing/augmenting the hidden spacebar shortcut).
- **Modified:** `docker-compose.yml`
    - **Keep:** `blah2_web` service (Apache) on port 49152.
    - **Add:** `blah2_web_v2` service (Nginx) on port 49153.
        - Maps `./html_v2` to `/usr/share/nginx/html`.
    - Remove deprecated `version` field.
- **Modified:** `Dockerfile`
    - (Optional) Refactor to use a separate runtime stage to reduce image size.

[Functions]
- **Modified:** `html_v2/control.js` -> `toggleCapture()`
    - Extract the capture toggle logic into a named function that can be called by both the keypress event and the new UI button click.

[Classes]
N/A - No class-based changes in the frontend code (it uses functional JS).

[Dependencies]
- **New:** `nginx:alpine` Docker image (for the new `blah2_web_v2` service).
- **Existing:** Bootstrap 5 (already included, will be re-styled).
- **Existing:** Plotly.js (already included, config will be updated).

[Implementation Order]
1.  **Setup:** Create `html_v2` directory by copying `html`.
2.  **Docker Config:** Update `docker-compose.yml` to add the `blah2_web_v2` service on port 49153.
3.  **CSS Overhaul:** Rewrite `html_v2/lib/blah2.css` with the new dark theme variables and base styles.
4.  **HTML Structure:** Update `html_v2/index.html` to implement the dashboard layout and add the new control buttons.
5.  **JS Logic:** Update `html_v2/control.js` and `html_v2/js/plot_map.js` to apply the dark theme and wire up controls.
6.  **Final Polish:** Verify the look and feel on port 49153, ensuring the original site on 49152 remains untouched.

task_progress Items:
- [ ] Step 1: Create `html_v2` and update `docker-compose.yml`
- [ ] Step 2: Rewrite `html_v2/lib/blah2.css` with dark theme
- [ ] Step 3: Update `html_v2/index.html` with dashboard layout
- [ ] Step 4: Update `html_v2/control.js` and `html_v2/js/plot_map.js`
- [ ] Step 5: Verify both interfaces
