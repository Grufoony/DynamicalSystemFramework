// Initialize the Leaflet map
const baseZoom = 13;
const map = L.map('map').setView([0, 0], 1);

// Grufoony - 9/2/2026
// TODO: make this dynamic based on data range
const MAX_DENSITY = 200;
const MAX_DENSITY_INVERTED = 1 / MAX_DENSITY;

// Add OpenStreetMap tile layer with inverted grayscale effect
const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>',
  crossOrigin: true,
  referrerPolicy: 'strict-origin-when-cross-origin'
}).addTo(map);
tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';

// Add scale control
L.control.scale({
  position: 'bottomright',
  metric: true,
  imperial: false
}).addTo(map);

// Add background menu control
L.Control.BackgroundMenu = L.Control.extend({
  options: {
    position: 'topright'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-settings');
    container.style.position = 'relative';

    const settingsBtn = L.DomUtil.create('button', 'leaflet-control-search-btn', container);
    settingsBtn.innerHTML = '⚙️';
    settingsBtn.title = 'Background Options';
    settingsBtn.onclick = () => {
      const settingsMenu = container.querySelector('.settings');
      if (settingsMenu.style.display === 'block') {
        settingsMenu.style.display = 'none';
      } else {
        settingsMenu.style.display = 'block';
        settingsMenu.style.position = 'absolute';
        settingsMenu.style.top = `${settingsBtn.offsetTop}px`;
        settingsMenu.style.right = `${container.offsetWidth - settingsBtn.offsetLeft + settingsBtn.offsetWidth}px`;
        settingsMenu.style.width = '100px';
        settingsMenu.style.zIndex = '1000';
      }
    };

    const settingsMenu = L.DomUtil.create('div', 'settings', container);
    settingsMenu.style.display = 'none';

    const normalBtn = L.DomUtil.create('button', '', settingsMenu);
    normalBtn.innerHTML = 'Normal';
    normalBtn.onclick = () => {
      tileLayer.getContainer().style.filter = '';
      settingsMenu.style.display = 'none';
    };

    const grayBtn = L.DomUtil.create('button', '', settingsMenu);
    grayBtn.innerHTML = 'Grayscale';
    grayBtn.onclick = () => {
      tileLayer.getContainer().style.filter = 'grayscale(100%)';
      settingsMenu.style.display = 'none';
    };

    const invertedBtn = L.DomUtil.create('button', '', settingsMenu);
    invertedBtn.innerHTML = 'Inverted';
    invertedBtn.onclick = () => {
      tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';
      settingsMenu.style.display = 'none';
    };

    return container;
  }
});

// Add search menu control
L.Control.SearchMenu = L.Control.extend({
  options: {
    position: 'topright'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-search');
    container.style.position = 'relative';

    const searchBtn = L.DomUtil.create('button', 'leaflet-control-search-btn', container);
    searchBtn.innerHTML = '🔍';
    searchBtn.title = 'Toggle Search Menu';
    searchBtn.onclick = () => {
      const searchContainer = document.querySelector('.search-container');
      if (searchContainer.style.display === 'block') {
        searchContainer.style.display = 'none';
      } else {
        searchContainer.style.display = 'block';
        searchContainer.style.position = 'absolute';
        searchContainer.style.top = `${searchBtn.offsetTop}px`;
        searchContainer.style.right = `${container.offsetWidth - searchBtn.offsetLeft + searchBtn.offsetWidth}px`;
        searchContainer.style.width = '200px';
        searchContainer.style.zIndex = '1000';
      }
    };

    return container;
  }
});

// Add background menu control to map
map.addControl(new L.Control.BackgroundMenu());

// Add search menu control to map
map.addControl(new L.Control.SearchMenu());

// Add chart toggle control
L.Control.ChartToggle = L.Control.extend({
  options: {
    position: 'topright'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-chart-toggle');
    const button = L.DomUtil.create('button', 'leaflet-control-search-btn', container);

    button.innerHTML = '📈';
    button.title = 'Toggle Chart';
    button.onclick = () => {
      const chartContainer = document.querySelector('.chart-container');
      if (chartContainer.style.display === 'none' || chartContainer.style.display === '') {
        chartContainer.style.display = 'block';
        if (typeof chart !== 'undefined' && chart) {
          chart.resize();
        }
      } else {
        chartContainer.style.display = 'none';
      }
    };

    return container;
  }
});

// Add chart toggle control to map
map.addControl(new L.Control.ChartToggle());

// Add screenshot control
L.Control.Screenshot = L.Control.extend({
  options: {
    position: 'topleft'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-screenshot');
    const button = L.DomUtil.create('a', 'leaflet-control-screenshot-button', container);

    button.innerHTML = '📷';
    button.href = '#';
    button.title = 'Take Screenshot';
    button.style.cssText = `
      width: 26px;
      height: 26px;
      line-height: 26px;
      display: block;
      text-align: center;
      text-decoration: none;
      color: black;
      background: white;
      border: 2px solid rgba(0,0,0,0.2);
      border-radius: 4px;
      box-shadow: 0 1px 5px rgba(0,0,0,0.4);
      font-size: 14px;
      margin-bottom: 5px;
    `;

    L.DomEvent.on(button, 'click', L.DomEvent.stopPropagation)
              .on(button, 'click', L.DomEvent.preventDefault)
              .on(button, 'click', this._takeScreenshot, this);

    return container;
  },

  _takeScreenshot: function() {
    const loadingDiv = document.createElement('div');
    loadingDiv.innerHTML = 'Generating screenshot...';
    loadingDiv.style.cssText = `
      position: fixed;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      background: rgba(0,0,0,0.8);
      color: white;
      padding: 20px;
      border-radius: 10px;
      z-index: 10000;
      font-size: 16px;
    `;
    document.body.appendChild(loadingDiv);

    const currentFilter = tileLayer.getContainer().style.filter;

    html2canvas(document.getElementById('app-container'), {
      useCORS: true,
      allowTaint: false,
      scale: 2,
      width: window.innerWidth,
      height: window.innerHeight,
      ignoreElements: (element) => {
        return element.classList.contains('data-selector') && element.style.display === 'none';
      },
      onclone: (clonedDoc) => {
        if (currentFilter && currentFilter !== 'none' && currentFilter !== '') {
          const clonedTileImages = clonedDoc.querySelectorAll('.leaflet-tile-pane img');
          const originalTileImages = document.querySelectorAll('.leaflet-tile-pane img');

          clonedTileImages.forEach((img, index) => {
            const originalImg = originalTileImages[index];
            if (originalImg && originalImg.complete) {
              const canvas = clonedDoc.createElement('canvas');
              canvas.width = originalImg.naturalWidth || originalImg.width;
              canvas.height = originalImg.naturalHeight || originalImg.height;
              canvas.style.cssText = img.style.cssText;
              canvas.className = img.className;
              const ctx = canvas.getContext('2d');
              ctx.filter = currentFilter;
              ctx.drawImage(originalImg, 0, 0, canvas.width, canvas.height);
              if (img.parentNode) {
                img.parentNode.replaceChild(canvas, img);
              }
            }
          });
        }
      }
    }).then(canvas => {
      document.body.removeChild(loadingDiv);

      const cropX = 0, cropY = 0, cropWidth = canvas.width, cropHeight = canvas.height;

      const { jsPDF } = window.jspdf;
      const pdf = new jsPDF({
        orientation: cropWidth > cropHeight ? 'landscape' : 'portrait',
        unit: 'px',
        format: [cropWidth, cropHeight]
      });

      const imgData = canvas.toDataURL('image/png');
      pdf.addImage(imgData, 'PNG', 0, 0, cropWidth, cropHeight);

      const filename = `road_network_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.pdf`;
      pdf.save(filename);
    }).catch(error => {
      document.body.removeChild(loadingDiv);
      console.error('Screenshot failed:', error);
      alert('Screenshot failed. Please try again.');
    });
  },

  _getScaleText: function() {
    const scaleElement = document.querySelector('.leaflet-control-scale-line');
    if (scaleElement) {
      return scaleElement.textContent;
    }
    return 'Scale information not available';
  }
});

// Add screenshot control to map
map.addControl(new L.Control.Screenshot());

// Add MP4 recording control
L.Control.MP4Recorder = L.Control.extend({
  options: {
    position: 'topleft'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-mp4');
    const button = L.DomUtil.create('a', 'leaflet-control-mp4-button', container);

    button.innerHTML = '🎥';
    button.href = '#';
    button.title = 'Record MP4';
    button.style.cssText = `
      width: 26px;
      height: 26px;
      line-height: 26px;
      display: block;
      text-align: center;
      text-decoration: none;
      color: black;
      background: white;
      border: 2px solid rgba(0,0,0,0.2);
      border-radius: 4px;
      box-shadow: 0 1px 5px rgba(0,0,0,0.4);
      font-size: 14px;
      margin-bottom: 5px;
    `;

    L.DomEvent.on(button, 'click', L.DomEvent.stopPropagation)
              .on(button, 'click', L.DomEvent.preventDefault)
              .on(button, 'click', this._startRecording, this);

    this.button = button;
    return container;
  },

  _startRecording: async function() {
    if (!densities || densities.length === 0) {
      alert('Please load data first.');
      return;
    }

    const playBtn = document.getElementById('playBtn');
    if (playBtn && playBtn.textContent === '⏸') {
      playBtn.click();
    }

    const fpsInput = document.getElementById('fpsInput');
    const fps = parseFloat(fpsInput.value) || 10;

    if (!confirm(`Start recording MP4 from current time to end?\nFPS: ${fps}\nNote: This process may take a while.`)) {
      return;
    }

    let isRecording = true;

    const loadingDiv = document.createElement('div');
    loadingDiv.id = 'mp4-progress';
    loadingDiv.style.cssText = `
      position: fixed;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      background: rgba(0,0,0,0.8);
      color: white;
      padding: 20px;
      border-radius: 10px;
      z-index: 10000;
      font-size: 16px;
      text-align: center;
    `;
    loadingDiv.innerHTML = 'Initializing MP4 recorder...<br>';

    const stopBtn = document.createElement('button');
    stopBtn.textContent = 'Stop & Save';
    stopBtn.style.cssText = 'margin-top: 10px; padding: 5px 10px; cursor: pointer;';
    stopBtn.onclick = () => {
      isRecording = false;
      stopBtn.disabled = true;
      stopBtn.textContent = 'Stopping...';
    };
    loadingDiv.appendChild(stopBtn);

    document.body.appendChild(loadingDiv);

    try {
      const { Muxer, ArrayBufferTarget } = await import('https://unpkg.com/mp4-muxer@5.1.4/build/mp4-muxer.mjs');

      const width = map.getSize().x;
      const height = map.getSize().y;
      const w = width % 2 === 0 ? width : width - 1;
      const h = height % 2 === 0 ? height : height - 1;

      const muxer = new Muxer({
        target: new ArrayBufferTarget(),
        video: {
          codec: 'avc',
          width: w,
          height: h
        },
        fastStart: 'in-memory'
      });

      const videoEncoder = new VideoEncoder({
        output: (chunk, meta) => muxer.addVideoChunk(chunk, meta),
        error: e => {
          console.error(e);
          alert("Video encoding error: " + e.message);
          isRecording = false;
        }
      });

      videoEncoder.configure({
        codec: 'avc1.4d002a',
        width: w,
        height: h,
        bitrate: 5_000_000
      });

      const timeSlider = document.getElementById('timeSlider');
      const startVal = parseInt(timeSlider.value);
      const maxVal = parseInt(timeSlider.max);
      const step = parseInt(timeSlider.step);

      let currentVal = startVal;
      let frameIndex = 0;

      const captureFrame = async () => {
        if (!isRecording || currentVal > maxVal) {
          loadingDiv.innerHTML = 'Finalizing MP4...';
          await videoEncoder.flush();
          muxer.finalize();

          const buffer = muxer.target.buffer;
          const blob = new Blob([buffer], { type: 'video/mp4' });
          const url = URL.createObjectURL(blob);

          const a = document.createElement('a');
          a.href = url;
          a.download = `simulation_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.mp4`;
          document.body.appendChild(a);
          a.click();
          document.body.removeChild(a);
          window.URL.revokeObjectURL(url);

          if (document.getElementById('mp4-progress')) {
            document.body.removeChild(loadingDiv);
          }
          return;
        }

        timeSlider.value = currentVal;
        timeSlider.dispatchEvent(new Event('input'));

        const progress = Math.round(((currentVal - startVal) / (maxVal - startVal)) * 100);
        loadingDiv.innerHTML = '';
        loadingDiv.appendChild(document.createTextNode(`Capturing frames: ${progress}%`));
        loadingDiv.appendChild(document.createElement('br'));
        loadingDiv.appendChild(document.createTextNode(`Time: ${document.getElementById('timeLabel').textContent}`));
        loadingDiv.appendChild(document.createElement('br'));
        loadingDiv.appendChild(stopBtn);

        await new Promise(resolve => setTimeout(resolve, 200));

        try {
          const canvas = await html2canvas(document.getElementById('app-container'), {
            useCORS: true,
            allowTaint: false,
            logging: false,
            scale: 1,
            width: w,
            height: h,
            ignoreElements: (element) => {
              return element.style.display === 'none';
            },
            onclone: (clonedDoc) => {
              const currentFilter = tileLayer.getContainer().style.filter;
              if (currentFilter && currentFilter !== 'none') {
                const originalImages = document.querySelectorAll('.leaflet-tile-pane img');
                const clonedImages = clonedDoc.querySelectorAll('.leaflet-tile-pane img');
                clonedImages.forEach((img, index) => {
                  const original = originalImages[index];
                  if (original && original.complete) {
                    try {
                      const canvas = document.createElement('canvas');
                      canvas.width = original.naturalWidth;
                      canvas.height = original.naturalHeight;
                      const ctx = canvas.getContext('2d');
                      ctx.filter = currentFilter;
                      ctx.drawImage(original, 0, 0);
                      img.src = canvas.toDataURL();
                      img.style.filter = 'none';
                    } catch (e) { /* silent */ }
                  }
                });
                const layers = clonedDoc.querySelectorAll('.leaflet-tile-pane .leaflet-layer');
                layers.forEach(layer => { layer.style.filter = 'none'; });
              }
            }
          });

          const frame = new VideoFrame(canvas, {
            timestamp: frameIndex * 1000000 / fps,
            duration: 1000000 / fps
          });

          videoEncoder.encode(frame, { keyFrame: frameIndex % 30 === 0 });
          frame.close();

          currentVal += step;
          frameIndex++;
          setTimeout(captureFrame, 0);
        } catch (err) {
          console.error(err);
          alert('Error capturing frame: ' + err.message);
          if (document.getElementById('mp4-progress')) {
            document.body.removeChild(loadingDiv);
          }
        }
      };

      captureFrame();

    } catch (err) {
      console.error(err);
      alert('Error initializing MP4 recorder: ' + err.message);
      if (document.getElementById('mp4-progress')) {
        document.body.removeChild(loadingDiv);
      }
    }
  }
});

// Add MP4 recorder control to map
map.addControl(new L.Control.MP4Recorder());

function valueToRgba(value, domainMin, domainMax, reversed) {
  const range = domainMax - domainMin;
  let t = range === 0 ? 0 : (value - domainMin) / range;
  t = Math.max(0, Math.min(1, t));
  if (reversed) t = 1 - t;

  let r, g;
  if (t <= 0.5) {
    const s = t * 2;           // 0 → 1 over first half
    r = Math.round(s * 255);   // 0 → 255
    g = Math.round(128 + s * 127); // 128 → 255
  } else {
    const s = (t - 0.5) * 2;  // 0 → 1 over second half
    r = 255;
    g = Math.round((1 - s) * 255); // 255 → 0
  }
  return `rgba(${r},${g},0,0.69)`;
}

function precomputeAllColors(observableData, observableDomains) {
  const result = {};
  for (const [key, rows] of Object.entries(observableData)) {
    const [domainMin, domainMax] = observableDomains[key] || [0, 1];
    const reversed = (EDGE_OBSERVABLE_CONFIG[key] || {}).reverseColorScale || false;
    result[key] = rows.map(row =>
      row.values.map(value => {
        const v = (value == null || isNaN(value)) ? 0 : +value;
        return valueToRgba(v, domainMin, domainMax, reversed);
      })
    );
  }
  return result;
}

// ============================================================
// Custom Canvas layer for edges
// ============================================================
L.CanvasEdges = L.Layer.extend({
  initialize: function(edges, options) {
    L.setOptions(this, options);
    this.edges = edges;
    this.colors = [];
    this.densities = [];

    // Stores latLngToLayerPoint results as Float32Array per edge.
    // Layer coordinates are stable across pans (only change on zoom),
    // so we project once per zoom level and reuse on every pan/redraw.
    //
    // Drawing formula: containerX = layerX + map._getMapPanePos().x
    //                  containerY = layerY + map._getMapPanePos().y
    // mapPanePos is a cheap JS property read, not a spherical math call.
    this._projectedGeometries = null; // Float32Array[] in layer coords
    this._cachedZoom = null;

    // Edges sorted by (quantized width, color string) so the draw loop
    // can batch consecutive same-style edges into one ctx.stroke() call.
    // Rebuilt when colors change (setColors) or zoom changes.
    this._sortedIndices = null;
    this._needsResort = true;

    // Hit-testing runs at most once per animation frame.
    this._pendingMouseMove = false;
    this._lastMouseEvent = null;
  },

  onAdd: function(map) {
    this._map = map;
    this._canvas = L.DomUtil.create('canvas', 'leaflet-canvas-layer');
    this._ctx = this._canvas.getContext('2d');

    this._canvas.style.pointerEvents = 'none';
    this._canvas.style.position = 'absolute';
    this._canvas.style.top = '0';
    this._canvas.style.left = '0';

    map.getPanes().overlayPane.appendChild(this._canvas);

    map.on('viewreset', this._reset, this);
    map.on('zoom', this._update, this);
    map.on('zoomstart', this._onZoomStart, this);
    map.on('zoomend', this._onZoomEnd, this);
    map.on('move', this._update, this);
    map.on('moveend', this._update, this);
    map.on('click', this._onMapClick, this);
    map.on('mousemove', this._onMouseMove, this);

    this._reset();
  },

  onRemove: function(map) {
    map.getPanes().overlayPane.removeChild(this._canvas);
    map.off('viewreset', this._reset, this);
    map.off('zoom', this._update, this);
    map.off('zoomstart', this._onZoomStart, this);
    map.off('zoomend', this._onZoomEnd, this);
    map.off('move', this._update, this);
    map.off('moveend', this._update, this);
    map.off('click', this._onMapClick, this);
    map.off('mousemove', this._onMouseMove, this);

    if (this._zoomAnimationFrame) {
      cancelAnimationFrame(this._zoomAnimationFrame);
    }
  },

  _reset: function() {
    const size = this._map.getSize();
    this._canvas.width = size.x;
    this._canvas.height = size.y;

    const topLeft = this._map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this._canvas, topLeft);

    // Invalidate geometry cache: canvas size may have changed
    this._projectedGeometries = null;
    this._cachedZoom = null;

    this._update();
  },

  _update: function() {
    if (!this._map) return;

    const topLeft = this._map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this._canvas, topLeft);

    this._redraw();
  },

  _onZoomStart: function() {
    this._zooming = true;
    this._ctx.clearRect(0, 0, this._canvas.width, this._canvas.height);
  },

  _onZoomEnd: function() {
    this._zooming = false;

    // Zoom changes pixel origin → layer coordinates change → invalidate cache
    this._projectedGeometries = null;
    this._cachedZoom = null;
    // Line widths also depend on zoom → resort
    this._needsResort = true;

    if (this._zoomAnimationFrame) {
      cancelAnimationFrame(this._zoomAnimationFrame);
      this._zoomAnimationFrame = null;
    }
    this._update();
  },

  _animateZoomUpdate: function() {
    if (!this._zooming) return;
    const topLeft = this._map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this._canvas, topLeft);
    this._zoomAnimationFrame = requestAnimationFrame(() => {
      this._animateZoomUpdate();
    });
  },

  // Layer coordinates = project(latlng) − pixelOrigin.
  // They are stable across pans and only change on zoom, so we cache them.
  _projectGeometries: function() {
    const zoom = this._map.getZoom();
    if (zoom === this._cachedZoom && this._projectedGeometries) return;

    this._projectedGeometries = this.edges.map(edge => {
      if (!edge.geometry || edge.geometry.length === 0) return null;
      // Store as flat Float32Array: [x0, y0, x1, y1, ...]
      // Float32Array is more cache-friendly than an array of objects.
      const arr = new Float32Array(edge.geometry.length * 2);
      for (let i = 0; i < edge.geometry.length; i++) {
        const lp = this._map.latLngToLayerPoint([edge.geometry[i].y, edge.geometry[i].x]);
        arr[i * 2]     = lp.x;
        arr[i * 2 + 1] = lp.y;
      }
      return arr;
    });
    this._cachedZoom = zoom;
  },

  setColors: function(colors) {
    this.colors = colors;
    this._needsResort = true; // Colors changed → resort
    this._redraw();
  },

  setDensities: function(densities) {
    this.densities = densities;
    this._needsResort = true; // Widths may change → resort
    this._redraw();
  },

  setHighlightedEdge: function(highlightedEdge) {
    this.highlightedEdge = highlightedEdge;
    this._redraw();
  },

  _redraw: function() {
    if (!this._map) return;
    if (this._zooming) return;

    // Ensure geometry cache is current (no-op if zoom unchanged)
    this._projectGeometries();

    const ctx = this._ctx;
    ctx.clearRect(0, 0, this._canvas.width, this._canvas.height);

    const zoom = this._map.getZoom();
    const baseStrokeWidth = 3 + (zoom - baseZoom);

    // containerPoint = layerPoint + mapPanePos
    // mapPanePos is a cheap property read (no math), changes on pan.
    const panePos = this._map._getMapPanePos();
    const ppx = panePos.x;
    const ppy = panePos.y;

    const nEdges = this.edges.length;

    // Precompute line widths for all edges (zoom + density dependent)
    const widths = new Float32Array(nEdges);
    for (let i = 0; i < nEdges; i++) {
      const density = (this.densities[i] || 0) * MAX_DENSITY_INVERTED;
      const densityFactor = Math.min(density, 2.0);
      widths[i] = Math.max(1, baseStrokeWidth * (0.5 + densityFactor));
    }

    // Sort by (quantized width, color string) so consecutive edges in the
    // loop share the same canvas state → fewer beginPath/stroke calls.
    // We cache this until colors or zoom changes (_needsResort).
    if (this._needsResort) {
      const indices = Array.from({ length: nEdges }, (_, i) => i);
      indices.sort((a, b) => {
        // Quantize width to 0.5 px buckets to widen groupings
        const wa = Math.round(widths[a] * 2);
        const wb = Math.round(widths[b] * 2);
        if (wa !== wb) return wa - wb;
        const ca = this.colors[a] || '';
        const cb = this.colors[b] || '';
        return ca < cb ? -1 : ca > cb ? 1 : 0;
      });
      this._sortedIndices = indices;
      this._needsResort = false;
    }

    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    // ---- Main draw pass: batched by (width, color) ----
    // We accumulate subpaths (moveTo/lineTo sequences) into a single
    // beginPath and only call stroke() when the style changes.
    // This reduces GPU state-change overhead from O(N) to O(unique styles).
    let currentColor = null;
    let currentQWidth = -1;
    let hasOpenPath = false;

    for (let si = 0; si < this._sortedIndices.length; si++) {
      const index = this._sortedIndices[si];
      const edge = this.edges[index];

      // Highlighted edge is drawn last (on top), skip here
      if (this.highlightedEdge && edge.id === this.highlightedEdge) continue;

      const pts = this._projectedGeometries[index];
      if (!pts || pts.length < 4) continue; // need ≥ 2 points

      const color  = this.colors[index] || 'rgba(0,128,0,0.69)';
      const qWidth = Math.round(widths[index] * 2) / 2; // quantize to 0.5 px

      if (color !== currentColor || qWidth !== currentQWidth) {
        // Flush current batch and open a new one
        if (hasOpenPath) ctx.stroke();
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth   = qWidth;
        currentColor  = color;
        currentQWidth = qWidth;
        hasOpenPath   = false;
      }

      // Accumulate this edge's geometry into the current path
      const nPts = pts.length >> 1; // pts.length / 2
      ctx.moveTo(pts[0] + ppx, pts[1] + ppy);
      for (let i = 1; i < nPts; i++) {
        ctx.lineTo(pts[i * 2] + ppx, pts[i * 2 + 1] + ppy);
      }
      hasOpenPath = true;
    }
    if (hasOpenPath) ctx.stroke(); // flush final batch

    // ---- Autostrada dashed overlay ----
    // Drawn as a second pass so the dashes sit on top of the solid strokes.
    ctx.setLineDash([4, 4]);
    for (let index = 0; index < nEdges; index++) {
      const edge = this.edges[index];
      if (!edge.name.toLowerCase().includes('autostrada')) continue;
      const pts = this._projectedGeometries[index];
      if (!pts || pts.length < 4) continue;

      ctx.beginPath();
      ctx.strokeStyle = this.colors[index] || 'rgba(0,128,0,0.69)';
      ctx.lineWidth   = widths[index];
      const nPts = pts.length >> 1;
      ctx.moveTo(pts[0] + ppx, pts[1] + ppy);
      for (let i = 1; i < nPts; i++) {
        ctx.lineTo(pts[i * 2] + ppx, pts[i * 2 + 1] + ppy);
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // ---- Highlighted edge (always on top, white) ----
    if (this.highlightedEdge) {
      const hIdx = this.edges.findIndex(e => e.id === this.highlightedEdge);
      if (hIdx >= 0) {
        const pts = this._projectedGeometries[hIdx];
        if (pts && pts.length >= 4) {
          ctx.beginPath();
          ctx.strokeStyle = 'white';
          ctx.lineWidth   = widths[hIdx] * 1.5;
          const nPts = pts.length >> 1;
          ctx.moveTo(pts[0] + ppx, pts[1] + ppy);
          for (let i = 1; i < nPts; i++) {
            ctx.lineTo(pts[i * 2] + ppx, pts[i * 2 + 1] + ppy);
          }
          ctx.stroke();
        }
      }
    }
  },

  // ---- Optimized point-to-segment distance (no object allocation) ----
  _segmentDist: function(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay;
    const lenSq = dx * dx + dy * dy;
    let param = lenSq === 0 ? -1 : ((px - ax) * dx + (py - ay) * dy) / lenSq;
    let xx, yy;
    if      (param < 0) { xx = ax; yy = ay; }
    else if (param > 1) { xx = bx; yy = by; }
    else                { xx = ax + param * dx; yy = ay + param * dy; }
    const ex = px - xx, ey = py - yy;
    return Math.sqrt(ex * ex + ey * ey);
  },

  // Keep original signature for any external callers
  _pointToLineDistancePixels: function(point, lineStart, lineEnd) {
    return this._segmentDist(point.x, point.y, lineStart.x, lineStart.y, lineEnd.x, lineEnd.y);
  },

  _onMapClick: function(e) {
    if (!this._projectedGeometries) return;

    const cp = this._map.latLngToContainerPoint(e.latlng);
    const panePos = this._map._getMapPanePos();
    // Convert to layer space so we can compare against cached layer coords
    const mx = cp.x - panePos.x;
    const my = cp.y - panePos.y;

    let closestEdge = null;
    let minDist = Infinity;

    for (let index = 0; index < this.edges.length; index++) {
      const pts = this._projectedGeometries[index];
      if (!pts || pts.length < 4) continue;

      const nPts = pts.length >> 1;
      for (let i = 0; i < nPts - 1; i++) {
        const dist = this._segmentDist(
          mx, my,
          pts[i * 2],       pts[i * 2 + 1],
          pts[(i + 1) * 2], pts[(i + 1) * 2 + 1]
        );
        if (dist < minDist) {
          minDist = dist;
          closestEdge = this.edges[index];
        }
      }
    }

    if (closestEdge && minDist < 10) {
      highlightedEdge = closestEdge.id;
      highlightedNode = null;
      this.setHighlightedEdge(highlightedEdge);
      updateNodeHighlight();

      if (closestEdge.geometry && closestEdge.geometry.length > 0) {
        const lats = closestEdge.geometry.map(p => p.y);
        const lngs = closestEdge.geometry.map(p => p.x);
        const bounds = L.latLngBounds(
          [Math.min(...lats), Math.min(...lngs)],
          [Math.max(...lats), Math.max(...lngs)]
        );
        map.fitBounds(bounds, { padding: [20, 20] });
      }

      updateEdgeInfo(closestEdge);
      document.getElementById('inverseBtn').disabled = false;
    }
  },

  // Browser fires mousemove at 60-250 Hz; hit-testing all edges every time
  // is wasteful.  We latch the latest event and process it on the next
  // animation frame so we never run more than once per rendered frame.
  _onMouseMove: function(e) {
    this._lastMouseEvent = e;
    if (this._pendingMouseMove) return;
    this._pendingMouseMove = true;

    requestAnimationFrame(() => {
      this._pendingMouseMove = false;
      if (!this._projectedGeometries || !this._lastMouseEvent) return;

      const ev = this._lastMouseEvent;
      const cp = this._map.latLngToContainerPoint(ev.latlng);
      const panePos = this._map._getMapPanePos();
      const mx = cp.x - panePos.x;
      const my = cp.y - panePos.y;

      let minDist = Infinity;
      for (let index = 0; index < this.edges.length; index++) {
        const pts = this._projectedGeometries[index];
        if (!pts || pts.length < 4) continue;

        const nPts = pts.length >> 1;
        for (let i = 0; i < nPts - 1; i++) {
          const dist = this._segmentDist(
            mx, my,
            pts[i * 2],       pts[i * 2 + 1],
            pts[(i + 1) * 2], pts[(i + 1) * 2 + 1]
          );
          if (dist < minDist) minDist = dist;
        }
      }

      this._map.getContainer().style.cursor = minDist < 10 ? 'pointer' : '';
    });
  }
});

// Create an overlay for D3 visualizations (keeping for node highlights)
L.svg().addTo(map);
const overlay = d3.select(map.getPanes().overlayPane).select('svg');
const g = overlay.append('g').attr('class', 'leaflet-zoom-hide');

let edges, densities, globalData;
let timeStamp = new Date();
let highlightedEdge = null;
let highlightedNode = null;
let chart;
let db = null;
let selectedSimulationId = null;

// Precomputed color lookup table: { [observableKey]: colorString[][] }
// Populated once in initializeApp() after data is loaded.
let precomputedColors = {};

let edgeObservableData = {
  density: [],
  speed: [],
  traveltime: [],
  queue_length: []
};
let edgeObservableDomains = {
  density: [0, MAX_DENSITY],
  speed: [0, 1],
  traveltime: [0, 1],
  queue_length: [0, 1]
};
let selectedEdgeColorObservable = 'density';

const EDGE_OBSERVABLE_CONFIG = {
  density:      { label: 'Density' },
  speed:        { label: 'Speed',       reverseColorScale: true },
  traveltime:   { label: 'Travel Time' },
  queue_length: { label: 'Queue Length' }
};

function formatTime(date) {
  const year  = date.getFullYear();
  const month = (date.getMonth() + 1).toString().padStart(2, '0');
  const day   = date.getDate().toString().padStart(2, '0');
  const hours = date.getHours().toString().padStart(2, '0');
  const mins  = date.getMinutes().toString().padStart(2, '0');
  return `${year}-${month}-${day} ${hours}:${mins}`;
}

function updateNodeHighlight() {
  g.selectAll('.node-highlight').remove();
  if (highlightedNode) {
    const point = map.latLngToLayerPoint([highlightedNode.y, highlightedNode.x]);
    g.append('circle')
      .attr('class', 'node-highlight')
      .attr('cx', point.x)
      .attr('cy', point.y)
      .attr('r', 10)
      .attr('fill', 'white')
      .attr('stroke', 'white')
      .attr('stroke-width', 2);
  }
}

function updateEdgeInfo(edge) {
  const edgeIndex = edges.indexOf(edge);
  const currentDensityRow = densities.find(d => d.datetime.getTime() === timeStamp.getTime());
  let density = 'N/A';
  if (currentDensityRow) {
    density = currentDensityRow.densities[edgeIndex];
    if (density === undefined || isNaN(density)) density = 0;
    density = parseFloat(density).toFixed(2);
  }
  document.getElementById('searchResults').innerHTML = `
    <strong>Edge ID:</strong> ${edge.id}<br>
    <strong>Source:</strong> ${edge.source}<br>
    <strong>Target:</strong> ${edge.target}<br>
    <strong>Max Speed:</strong> ${edge.maxspeed || 'N/A'}<br>
    <strong>Name:</strong> ${edge.name}<br>
    <strong>Number of Lanes:</strong> ${edge.nlanes || 'N/A'}<br>
    <strong>Density:</strong> ${density}<br>
    <strong>Coil Code:</strong> ${edge.coilcode || 'N/A'}<br>
  `;
}

function parseGeometry(geometryStr) {
  if (!geometryStr) return [];
  const coordsStr = geometryStr.replace(/^LINESTRING\s*\(/, '').replace(/\)$/, '');
  return coordsStr.split(',').map(coordStr => {
    const coords = coordStr.trim().split(/\s+/);
    return { x: +coords[0], y: +coords[1] };
  });
}

function loadEdgesFromDB() {
  const result = db.exec('SELECT id, source, target, length, maxspeed, name, nlanes, geometry, coilcode FROM edges');
  if (result.length === 0) return [];

  const columns = result[0].columns;
  const values  = result[0].values;

  return values.map(row => {
    const edge = {};
    columns.forEach((col, i) => { edge[col] = row[i]; });
    edge.geometry = parseGeometry(edge.geometry);
    edge.maxspeed  = +edge.maxspeed  || 0;
    edge.nlanes    = +edge.nlanes    || 1;
    edge.length    = +edge.length    || 0;
    edge.coilcode  = edge.coilcode   || null;
    return edge;
  });
}

function getRoadDataColumns() {
  const result = db.exec('PRAGMA table_info(road_data)');
  if (result.length === 0) return [];
  return result[0].values.map(row => row[1]);
}

function getTravelTimeExpression() {
  const columns = getRoadDataColumns();
  if (columns.includes('traveltime'))    return 'r.traveltime';
  if (columns.includes('travel_time'))   return 'r.travel_time';
  if (columns.includes('travel_time_s')) return 'r.travel_time_s';
  return 'CASE WHEN r.avg_speed_kph > 0 THEN e.length / (r.avg_speed_kph / 3.6) ELSE 0 END';
}

function computeObservableDomain(observableRows) {
  const values = observableRows
    .flatMap(row => row.values)
    .map(v => +v)
    .filter(v => Number.isFinite(v));

  if (values.length === 0) return [0, 1];

  let minValue = values[0];
  let maxValue = values[0];
  for (let i = 1; i < values.length; i++) {
    if (values[i] < minValue) minValue = values[i];
    if (values[i] > maxValue) maxValue = values[i];
  }

  if (minValue === maxValue) maxValue = minValue + 1;
  return [minValue, maxValue];
}

function loadRoadDataFromDB() {
  const edgeIds = edges.map(e => e.id);
  const travelTimeExpression = getTravelTimeExpression();

  const result = db.exec(
    `SELECT r.datetime,
            r.street_id,
            r.density_vpk,
            r.avg_speed_kph,
            ${travelTimeExpression} AS traveltime,
            r.queue_length
     FROM road_data r
     LEFT JOIN edges e ON e.id = r.street_id
     WHERE r.simulation_id = ${selectedSimulationId}
     ORDER BY r.datetime, r.street_id`
  );

  if (result.length === 0) {
    return {
      densities: [],
      observables: {
        density: [], speed: [], traveltime: [], queue_length: []
      },
      domains: {
        density: [0, 1], speed: [0, 1], traveltime: [0, 1], queue_length: [0, 1]
      }
    };
  }

  const densityData      = [];
  const speedData        = [];
  const travelTimeData   = [];
  const queueLengthData  = [];

  let currentTs            = null;
  let currentDensityMap    = {};
  let currentSpeedMap      = {};
  let currentTravelTimeMap = {};
  let currentQueueLengthMap = {};

  const flush = (ts) => {
    densityData.push({
      datetime: new Date(ts),
      densities: edgeIds.map(id => +currentDensityMap[id] || 0)
    });
    speedData.push({
      datetime: new Date(ts),
      values: edgeIds.map(id => +currentSpeedMap[id] || 0)
    });
    travelTimeData.push({
      datetime: new Date(ts),
      values: edgeIds.map(id => +currentTravelTimeMap[id] || 0)
    });
    queueLengthData.push({
      datetime: new Date(ts),
      values: edgeIds.map(id => +currentQueueLengthMap[id] || 0)
    });
  };

  for (const row of result[0].values) {
    const [ts, streetId, density, speed, travelTime, queueLength] = row;

    if (ts !== currentTs) {
      if (currentTs !== null) flush(currentTs);
      currentTs = ts;
      currentDensityMap     = {};
      currentSpeedMap       = {};
      currentTravelTimeMap  = {};
      currentQueueLengthMap = {};
    }

    currentDensityMap[streetId]     = density;
    currentSpeedMap[streetId]       = speed;
    currentTravelTimeMap[streetId]  = travelTime;
    currentQueueLengthMap[streetId] = queueLength;
  }
  if (currentTs !== null) flush(currentTs);

  const observables = {
    density:      densityData.map(r => ({ datetime: r.datetime, values: r.densities })),
    speed:        speedData,
    traveltime:   travelTimeData,
    queue_length: queueLengthData
  };

  const domains = {
    density:      computeObservableDomain(observables.density),
    speed:        computeObservableDomain(observables.speed),
    traveltime:   computeObservableDomain(observables.traveltime),
    queue_length: computeObservableDomain(observables.queue_length)
  };

  return { densities: densityData, observables, domains };
}

function loadGlobalDataFromDB() {
  const tablesResult = db.exec("SELECT name FROM sqlite_master WHERE type='table'");
  const tableNames = tablesResult.length > 0 ? tablesResult[0].values.map(r => r[0]) : [];

  const avgStatsTable = tableNames.includes('avg_stats')
    ? 'avg_stats'
    : (tableNames.includes('avgstats') ? 'avgstats' : null);

  if (!avgStatsTable) {
    const result = db.exec(`
      SELECT datetime,
             AVG(density_vpk) as mean_density_vpk,
             AVG(avg_speed_kph) as mean_speed_kph,
             SUM(counts) as total_counts
      FROM road_data
      WHERE simulation_id = ${selectedSimulationId}
      GROUP BY datetime
      ORDER BY datetime
    `);
    if (result.length === 0) return [];
    return result[0].values.map(row => {
      const data = { datetime: new Date(row[0]) };
      result[0].columns.slice(1).forEach((col, i) => { data[col] = +row[i + 1] || 0; });
      return data;
    });
  }

  const colsResult = db.exec(`PRAGMA table_info(${avgStatsTable})`);
  if (colsResult.length === 0) return [];

  const allColumns = colsResult[0].values.map(row => row[1]);
  const metricColumns = allColumns.filter(
    col => !['id', 'simulation_id', 'datetime', 'time_step'].includes(col)
  );
  if (metricColumns.length === 0) return [];

  const result = db.exec(`
    SELECT datetime, ${metricColumns.join(', ')}
    FROM ${avgStatsTable}
    WHERE simulation_id = ${selectedSimulationId}
    ORDER BY datetime
  `);
  if (result.length === 0) return [];

  return result[0].values.map(row => {
    const data = { datetime: new Date(row[0]) };
    metricColumns.forEach((col, i) => { data[col] = +row[i + 1] || 0; });
    return data;
  });
}

function getSimulations() {
  const result = db.exec('SELECT id, name FROM simulation_info ORDER BY id');
  if (result.length === 0) return [];
  return result[0].values.map(row => ({
    id: row[0],
    name: row[1] || `Simulation ${row[0]}`
  }));
}

function initializeApp() {
  edges = loadEdgesFromDB();
  const roadDataBundle = loadRoadDataFromDB();
  densities             = roadDataBundle.densities;
  edgeObservableData    = roadDataBundle.observables;
  edgeObservableDomains = roadDataBundle.domains;
  globalData            = loadGlobalDataFromDB();

  console.log('Loaded edges:', edges.length);
  console.log('Loaded density timestamps:', densities.length);
  console.log('Loaded global data:', globalData.length);

  if (!edges.length)    { alert('No edges found in database.'); return; }
  if (!densities.length){ alert(`No road_data found for simulation ID ${selectedSimulationId}.`); return; }

  timeStamp = densities[0].datetime;

  // This runs once and takes O(observables × timesteps × edges) time.
  console.time('precomputeAllColors');
  precomputedColors = precomputeAllColors(edgeObservableData, edgeObservableDomains);
  console.timeEnd('precomputeAllColors');

  // Center map on median of all geometry points
  const allLats = [], allLons = [];
  edges.forEach(edge => {
    if (edge.geometry) edge.geometry.forEach(pt => { allLats.push(pt.y); allLons.push(pt.x); });
  });
  if (allLats.length > 0) {
    allLats.sort((a, b) => a - b);
    allLons.sort((a, b) => a - b);
    map.setView(
      [allLats[Math.floor(allLats.length / 2)], allLons[Math.floor(allLons.length / 2)]],
      baseZoom
    );
  }

  // Create canvas layer
  const canvasEdges = new L.CanvasEdges(edges);
  canvasEdges.addTo(map);

  const edgeColorObservableSelector = document.getElementById('edgeColorObservableSelector');
  selectedEdgeColorObservable = edgeColorObservableSelector.value || 'density';

  function formatLegendValue(value) {
    const n = +value;
    if (!Number.isFinite(n)) return 'N/A';
    if (Math.abs(n) >= 100) return n.toFixed(0);
    if (Math.abs(n) >= 10)  return n.toFixed(1);
    return n.toFixed(2);
  }

  function updateLegend() {
    const config = EDGE_OBSERVABLE_CONFIG[selectedEdgeColorObservable] || EDGE_OBSERVABLE_CONFIG.density;
    const domain = edgeObservableDomains[selectedEdgeColorObservable] || [0, 1];
    const mid    = (domain[0] + domain[1]) / 2;

    document.querySelector('.legend-title').textContent = config.label;

    const legendBar = document.querySelector('.legend-bar');
    if (legendBar) {
      legendBar.style.background = config.reverseColorScale
        ? 'linear-gradient(to right, red, yellow, green)'
        : 'linear-gradient(to right, green, yellow, red)';
    }

    const labels = document.querySelectorAll('.legend-labels span');
    if (labels.length >= 3) {
      labels[0].textContent = formatLegendValue(domain[0]);
      labels[1].textContent = formatLegendValue(mid);
      labels[2].textContent = formatLegendValue(domain[1]);
    }
  }

  edgeColorObservableSelector.onchange = () => {
    selectedEdgeColorObservable = edgeColorObservableSelector.value;
    updateLegend();
    updateDensityVisualization();
  };

  let currentChartColumn = 'mean_density_vpk';

  if (globalData && globalData.length > 0) {
    const columns  = Object.keys(globalData[0]).filter(k => k !== 'datetime');
    const selector = document.getElementById('chartColumnSelector');
    selector.innerHTML = '';
    columns.forEach(col => {
      const opt = document.createElement('option');
      opt.value = col;
      opt.text  = col;
      selector.appendChild(opt);
    });

    if (columns.includes('mean_density_vpk')) {
      selector.value = 'mean_density_vpk';
    } else if (columns.length > 0) {
      selector.value = columns[0];
    }
    currentChartColumn = selector.value;

    selector.onchange = () => {
      currentChartColumn = selector.value;
      initChart();
      updateChart();
    };

    initChart();
  }

  function initChart() {
    const canvas = document.getElementById('densityChart');
    const ctx    = canvas.getContext('2d');
    if (chart) chart.destroy();

    chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: globalData.map(d => formatTime(d.datetime)),
        datasets: [
          {
            label: currentChartColumn,
            data: globalData.map(d => d[currentChartColumn]),
            borderColor: 'blue',
            borderWidth: 1,
            pointRadius: 0,
            fill: false,
            tension: 0.1
          },
          {
            label: 'Current Time',
            data: [],
            borderColor: 'red',
            backgroundColor: 'red',
            pointRadius: 5,
            pointHoverRadius: 7,
            showLine: false
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 },
        scales: {
          x: {
            display: true,
            title: { display: true, text: 'time' },
            ticks: { display: false }
          },
          y: {
            beginAtZero: true,
            title: { display: true, text: currentChartColumn }
          }
        },
        plugins: {
          legend: { display: true, labels: { boxWidth: 10 } }
        }
      }
    });

    let isDragging = false;

    const updateTimeFromEvent = (e) => {
      const points = chart.getElementsAtEventForMode(e, 'index', { intersect: false }, true);
      if (points.length) {
        const index = points[0].index;
        const timeSlider = document.getElementById('timeSlider');
        let currentDt = 300;
        if (densities.length > 1) {
          currentDt = Math.round((densities[1].datetime - densities[0].datetime) / 1000);
          if (currentDt <= 0) currentDt = 300;
        }
        timeSlider.value = index * currentDt;
        timeSlider.dispatchEvent(new Event('input'));
      }
    };

    canvas.onmousedown = (e) => { isDragging = true; updateTimeFromEvent(e); };
    canvas.onmousemove = (e) => { if (isDragging) updateTimeFromEvent(e); };
    canvas.onmouseup   = ()  => { isDragging = false; };
    canvas.onmouseleave = () => { isDragging = false; };
  }

  function updateChart() {
    if (!chart || !globalData) return;
    const currentIndex = densities.findIndex(d => d.datetime.getTime() === timeStamp.getTime());
    if (currentIndex !== -1) {
      const pointData = new Array(globalData.length).fill(null);
      if (globalData[currentIndex]) {
        pointData[currentIndex] = globalData[currentIndex][currentChartColumn];
      }
      chart.data.datasets[1].data = pointData;
      chart.update('none');
    }
  }

  function update() {
    updateDensityVisualization();
    updateNodeHighlight();
    updateChart();
  }

  map.on('zoomend', update);

  function updateDensityVisualization() {
    const currentIndex = densities.findIndex(d => d.datetime.getTime() === timeStamp.getTime());
    if (currentIndex < 0) {
      console.error('No road data for time step:', timeStamp);
      return;
    }

    // O(1) lookup per frame instead of O(N) D3 pipeline
    const colorArray = precomputedColors[selectedEdgeColorObservable]?.[currentIndex];
    if (!colorArray) return;

    canvasEdges.setColors(colorArray);
    canvasEdges.setDensities(densities[currentIndex].densities);
  }

  updateLegend();
  update();

  // Time slider
  const timeSlider = document.getElementById('timeSlider');
  const timeLabel  = document.getElementById('timeLabel');
  const playBtn    = document.getElementById('playBtn');
  const fpsInput   = document.getElementById('fpsInput');

  let isPlaying   = false;
  let playInterval = null;

  let dt = 300;
  if (densities.length > 1) {
    dt = Math.round((densities[1].datetime - densities[0].datetime) / 1000);
    if (dt <= 0) dt = 300;
  }
  timeSlider.max  = (densities.length - 1) * dt;
  timeSlider.step = dt;
  timeLabel.textContent = formatTime(timeStamp);

  function togglePlay() {
    isPlaying = !isPlaying;
    playBtn.textContent = isPlaying ? '⏸' : '▶';

    if (isPlaying) {
      const fps = parseFloat(fpsInput.value) || 10;
      playInterval = setInterval(() => {
        let currentValue = parseInt(timeSlider.value);
        const maxValue   = parseInt(timeSlider.max);
        currentValue = currentValue >= maxValue ? 0 : currentValue + dt;
        timeSlider.value = currentValue;
        timeSlider.dispatchEvent(new Event('input'));
      }, 1000 / fps);
    } else {
      clearInterval(playInterval);
      playInterval = null;
    }
  }

  playBtn.addEventListener('click', togglePlay);

  fpsInput.addEventListener('change', () => {
    if (isPlaying) { togglePlay(); togglePlay(); }
  });

  timeSlider.addEventListener('input', function() {
    const index = Math.floor(parseInt(timeSlider.value) / dt);
    timeStamp = densities[index].datetime;
    timeLabel.textContent = formatTime(timeStamp);
    update();
    if (highlightedEdge) {
      const edge = edges.find(e => e.id === highlightedEdge);
      if (edge) updateEdgeInfo(edge);
    }
  });

  // Edge search
  const edgeSearchBtn = document.getElementById('edgeSearchBtn');
  edgeSearchBtn.addEventListener('click', () => {
    const id   = document.getElementById('edgeSearch').value.trim();
    const edge = edges.find(e => e.id == id);
    if (edge) {
      highlightedEdge = id;
      canvasEdges.setHighlightedEdge(highlightedEdge);
      if (edge.geometry && edge.geometry.length > 0) {
        const lats = edge.geometry.map(p => p.y);
        const lngs = edge.geometry.map(p => p.x);
        map.fitBounds(
          L.latLngBounds([Math.min(...lats), Math.min(...lngs)], [Math.max(...lats), Math.max(...lngs)]),
          { padding: [20, 20] }
        );
      }
      updateEdgeInfo(edge);
      document.getElementById('inverseBtn').disabled = false;
    } else {
      document.getElementById('searchResults').innerHTML = 'Edge not found';
    }
  });

  // Node search
  const nodeSearchBtn = document.getElementById('nodeSearchBtn');
  nodeSearchBtn.addEventListener('click', () => {
    const id          = document.getElementById('nodeSearch').value.trim();
    const edgeAsSource = edges.find(e => e.source === id);
    const edgeAsTarget = edges.find(e => e.target === id);

    if (edgeAsSource) {
      const geom = edgeAsSource.geometry;
      if (geom && geom.length > 0) {
        highlightedNode = geom[0];
        updateNodeHighlight();
        map.setView([highlightedNode.y, highlightedNode.x], 18);
        document.getElementById('searchResults').innerHTML = `
          <strong>Node ID:</strong> ${id}<br>
          <strong>Position:</strong> (${highlightedNode.x}, ${highlightedNode.y})
        `;
      }
    } else if (edgeAsTarget) {
      const geom = edgeAsTarget.geometry;
      if (geom && geom.length > 0) {
        highlightedNode = geom[geom.length - 1];
        updateNodeHighlight();
        map.setView([highlightedNode.y, highlightedNode.x], 18);
        document.getElementById('searchResults').innerHTML = `
          <strong>Node ID:</strong> ${id}<br>
          <strong>Position:</strong> (${highlightedNode.x}, ${highlightedNode.y})
        `;
      }
    } else {
      document.getElementById('searchResults').innerHTML = 'Node not found';
    }
    document.getElementById('inverseBtn').disabled = true;
  });

  document.getElementById('edgeSearch').addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { e.preventDefault(); edgeSearchBtn.click(); }
  });

  document.getElementById('nodeSearch').addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { e.preventDefault(); nodeSearchBtn.click(); }
  });

  // Clear
  document.getElementById('clearBtn').addEventListener('click', () => {
    highlightedEdge = null;
    highlightedNode = null;
    canvasEdges.setHighlightedEdge(null);
    updateNodeHighlight();
    document.getElementById('searchResults').innerHTML = '';
    document.getElementById('edgeSearch').value = '';
    document.getElementById('nodeSearch').value = '';
    document.getElementById('inverseBtn').disabled = true;
  });

  // Inverse edge
  document.getElementById('inverseBtn').addEventListener('click', () => {
    if (!highlightedEdge) return;
    const currentEdge = edges.find(e => e.id === highlightedEdge);
    if (!currentEdge) return;

    const inverseEdge = edges.find(e => e.source === currentEdge.target && e.target === currentEdge.source);
    if (inverseEdge) {
      highlightedEdge = inverseEdge.id;
      highlightedNode = null;
      canvasEdges.setHighlightedEdge(highlightedEdge);
      updateNodeHighlight();
      if (inverseEdge.geometry && inverseEdge.geometry.length > 0) {
        const lats = inverseEdge.geometry.map(p => p.y);
        const lngs = inverseEdge.geometry.map(p => p.x);
        map.fitBounds(
          L.latLngBounds([Math.min(...lats), Math.min(...lngs)], [Math.max(...lats), Math.max(...lngs)]),
          { padding: [20, 20] }
        );
      }
      updateEdgeInfo(inverseEdge);
    } else {
      alert(`Inverse edge from ${currentEdge.target} to ${currentEdge.source} not found`);
    }
  });

  // Show UI
  document.querySelector('.slider-container').style.display = 'block';
  document.querySelector('.legend-container').style.display = 'block';
  if (globalData.length > 0) {
    document.querySelector('.chart-container').style.display = 'block';
  }
}

// Database loading and simulation selection via modal
document.addEventListener('DOMContentLoaded', () => {
  const dbFileInput = document.getElementById('dbFileInput');
  const loadDbBtn   = document.getElementById('loadDbBtn');
  const dbStatus    = document.getElementById('db-status');

  loadDbBtn.addEventListener('click', async () => {
    const file = dbFileInput.files[0];
    if (!file) {
      dbStatus.className = 'db-status error';
      dbStatus.textContent = 'Please select a database file.';
      return;
    }

    dbStatus.className = 'db-status loading';
    dbStatus.textContent = 'Loading database...';
    loadDbBtn.disabled = true;

    try {
      const fileSnapshot  = file.slice(0, file.size);
      const arrayBuffer   = await fileSnapshot.arrayBuffer();
      const uint8Array    = new Uint8Array(arrayBuffer);

      const SQL = await initSqlJs({
        locateFile: file => `https://cdnjs.cloudflare.com/ajax/libs/sql.js/1.10.3/${file}`
      });

      db = new SQL.Database(uint8Array);

      const tables     = db.exec("SELECT name FROM sqlite_master WHERE type='table'");
      const tableNames = tables.length > 0 ? tables[0].values.map(r => r[0]) : [];

      if (!tableNames.includes('edges'))      throw new Error("Database missing 'edges' table");
      if (!tableNames.includes('road_data'))  throw new Error("Database missing 'road_data' table");
      if (!tableNames.includes('simulation_info')) throw new Error("Database missing 'simulation_info' table");

      const simulations = getSimulations();
      if (simulations.length === 0) throw new Error('No simulations found in database');

      dbStatus.className = 'db-status success';
      dbStatus.textContent = `Database loaded! Found ${simulations.length} simulation(s).`;

      setTimeout(() => { showSimulationSelector(simulations); }, 500);

    } catch (error) {
      console.error('Database loading error:', error);
      dbStatus.className = 'db-status error';
      if (error instanceof DOMException && error.name === 'NotReadableError') {
        dbStatus.textContent = 'Error: Could not read the selected file. Re-select it, or move it to a local folder you own and try again.';
      } else {
        dbStatus.textContent = `Error: ${error.message}`;
      }
      loadDbBtn.disabled = false;
    }
  });

  function showSimulationSelector(simulations) {
    const modalContent = document.querySelector('.modal-content');
    modalContent.innerHTML = `
      <h2>Select Simulation</h2>
      <p>Choose which simulation to visualize:</p>
      <div class="db-input-group">
        <select id="simulationSelector" style="width:100%;padding:10px;font-size:16px;border:2px solid #ccc;border-radius:5px;">
          ${simulations.map(sim => `<option value="${sim.id}">${sim.name} (ID: ${sim.id})</option>`).join('')}
        </select>
      </div>
      <div id="db-status" class="db-status"></div>
      <button id="loadSimBtn" class="load-db-btn">Load Simulation</button>
    `;

    document.getElementById('loadSimBtn').addEventListener('click', () => {
      selectedSimulationId = parseInt(document.getElementById('simulationSelector').value);
      document.getElementById('db-modal').classList.add('hidden');
      initializeApp();
    });
  }
});