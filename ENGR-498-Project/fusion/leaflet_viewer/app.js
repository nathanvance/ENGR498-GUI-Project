const DEFAULT_DATA_PATH = "../calibration_test/fused_objects.json";

function getDataPaths() {
  const params = new URLSearchParams(window.location.search);
  const hasObjectPath = params.has("data");
  const hasPowerlinePath = params.has("powerlines");
  return {
    objectDataPath: hasObjectPath ? (params.get("data") || "") : (hasPowerlinePath ? "" : DEFAULT_DATA_PATH),
    powerlineDataPath: hasPowerlinePath ? (params.get("powerlines") || "") : ""
  };
}

function rgbString(rgb) {
  return `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
}

function isFiniteNumber(value) {
  return typeof value === "number" && Number.isFinite(value);
}

function formatMaybeNumber(value, digits = 3, fallback = "N/A") {
  return isFiniteNumber(value) ? Number(value).toFixed(digits) : fallback;
}

function escapeHtml(text) {
  return String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function featureDisplayName(feature) {
  return feature.object_name || feature.source_wire_key || feature.class_name || "feature";
}

function hasGpsPoint(feature) {
  return (
    feature.gps &&
    isFiniteNumber(feature.gps.lat) &&
    isFiniteNumber(feature.gps.lon)
  );
}

function parseGpsPolylinePoint(point) {
  if (Array.isArray(point) && point.length >= 2 && isFiniteNumber(point[0]) && isFiniteNumber(point[1])) {
    return L.latLng(point[0], point[1]);
  }
  if (
    point &&
    typeof point === "object" &&
    isFiniteNumber(point.lat) &&
    isFiniteNumber(point.lon)
  ) {
    return L.latLng(point.lat, point.lon);
  }
  return null;
}

function getGpsPolylineLatLngs(feature) {
  const points = feature.polyline_gps || [];
  const latLngs = [];
  for (const point of points) {
    const latLng = parseGpsPolylinePoint(point);
    if (latLng) {
      latLngs.push(latLng);
    }
  }
  return latLngs;
}

function getLocalPolylineLatLngs(feature) {
  const points = feature.polyline_map_xyz || [];
  const latLngs = [];
  for (const point of points) {
    if (Array.isArray(point) && point.length >= 2 && isFiniteNumber(point[0]) && isFiniteNumber(point[1])) {
      latLngs.push(L.latLng(point[1], point[0]));
    }
  }
  return latLngs;
}

function normalizeObjectFeature(object) {
  return {
    ...object,
    feature_kind: "object",
    score_value: object.confidence_score,
    score_label: "Confidence"
  };
}

function normalizePowerlineFeature(powerline) {
  return {
    ...powerline,
    class_name: powerline.class_name || "powerline",
    feature_kind: "powerline",
    score_value: powerline.quality_score,
    score_label: "R²",
    class_color_rgb: powerline.class_color_rgb || [95, 195, 235]
  };
}

function buildPopupHtml(feature) {
  const centroid = feature.centroid_map_xyz || [null, null, null];
  const bboxMin = feature.bbox_aabb_min_xyz || [null, null, null];
  const bboxMax = feature.bbox_aabb_max_xyz || [null, null, null];
  const gpsText = hasGpsPoint(feature)
    ? `${formatMaybeNumber(feature.gps.lat, 6)}, ${formatMaybeNumber(feature.gps.lon, 6)}`
    : "Not available";

  if (feature.feature_kind === "powerline") {
    const fit = feature.fit_metadata || {};
    const metrics = feature.metrics || {};
    return `
      <div>
        <strong>${escapeHtml(featureDisplayName(feature))}</strong><br>
        Class: ${escapeHtml(feature.class_name)}<br>
        Type: Powerline<br>
        Points: ${feature.num_points ?? "N/A"}<br>
        R²: ${formatMaybeNumber(fit.rsq ?? feature.quality_score, 3)}<br>
        Polyline Length: ${formatMaybeNumber(metrics.polyline_length_m, 2)} m<br>
        End-to-End Length: ${formatMaybeNumber(metrics.end_to_end_length_m, 2)} m<br>
        Sag: ${formatMaybeNumber(metrics.sag_m, 2)} m<br>
        Ground Clearance: ${formatMaybeNumber(metrics.ground_clearance_m, 2)} m<br>
        Centroid XYZ: ${formatMaybeNumber(centroid[0])}, ${formatMaybeNumber(centroid[1])}, ${formatMaybeNumber(centroid[2])}<br>
        GPS: ${gpsText}
      </div>
    `;
  }

  return `
    <div>
      <strong>${escapeHtml(featureDisplayName(feature))}</strong><br>
      Class: ${escapeHtml(feature.class_name)}<br>
      Confidence: ${formatMaybeNumber(feature.confidence_score, 2)}<br>
      Points: ${feature.num_points ?? "N/A"}<br>
      Centroid XYZ: ${formatMaybeNumber(centroid[0])}, ${formatMaybeNumber(centroid[1])}, ${formatMaybeNumber(centroid[2])}<br>
      BBox Min: ${formatMaybeNumber(bboxMin[0])}, ${formatMaybeNumber(bboxMin[1])}, ${formatMaybeNumber(bboxMin[2])}<br>
      BBox Max: ${formatMaybeNumber(bboxMax[0])}, ${formatMaybeNumber(bboxMax[1])}, ${formatMaybeNumber(bboxMax[2])}<br>
      GPS: ${gpsText}
    </div>
  `;
}

function createStatusBanner(text) {
  const banner = document.createElement("div");
  banner.className = "status-banner";
  banner.textContent = text;
  return banner;
}

async function fetchJson(path) {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(`Failed to load ${path}: ${response.status} ${response.statusText}`);
  }
  return response.json();
}

async function loadMapData() {
  const { objectDataPath, powerlineDataPath } = getDataPaths();
  document.getElementById("data-path").textContent = objectDataPath || "Not provided";
  document.getElementById("powerline-path").textContent = powerlineDataPath || "Not provided";

  const objectPayload = objectDataPath ? await fetchJson(objectDataPath) : { objects: [] };
  const powerlinePayload = powerlineDataPath ? await fetchJson(powerlineDataPath) : null;
  return { objectPayload, powerlinePayload, objectDataPath, powerlineDataPath };
}

function createLegend(features) {
  const legend = document.getElementById("legend");
  legend.innerHTML = "";

  const classes = new Map();
  for (const feature of features) {
    if (!classes.has(feature.class_name)) {
      classes.set(feature.class_name, feature.class_color_rgb || [100, 100, 100]);
    }
  }

  for (const [className, color] of classes.entries()) {
    const row = document.createElement("div");
    row.className = "legend-item";
    row.innerHTML = `
      <span class="swatch" style="background:${rgbString(color)}"></span>
      <span>${escapeHtml(className)}</span>
    `;
    legend.appendChild(row);
  }
}

function createFeatureList(features, onSelect) {
  const list = document.getElementById("object-list");
  const searchBox = document.getElementById("search-box");
  let activeCard = null;

  function render(filterText = "") {
    list.innerHTML = "";
    const query = filterText.trim().toLowerCase();
    const filtered = features.filter((feature) => {
      if (!query) {
        return true;
      }
      return (
        featureDisplayName(feature).toLowerCase().includes(query) ||
        String(feature.class_name || "").toLowerCase().includes(query)
      );
    });

    for (const feature of filtered) {
      const card = document.createElement("button");
      card.type = "button";
      card.className = "object-card";

      const metricText =
        feature.feature_kind === "powerline"
          ? `R² ${formatMaybeNumber(feature.score_value, 2)}`
          : formatMaybeNumber(feature.score_value, 2);

      const metaText =
        feature.feature_kind === "powerline"
          ? `powerline<br>Span: ${formatMaybeNumber(feature.metrics?.polyline_length_m, 2)} m`
          : `${escapeHtml(feature.class_name)}<br>Points: ${feature.num_points}`;

      card.innerHTML = `
        <div class="title-row">
          <span>${escapeHtml(featureDisplayName(feature))}</span>
          <span>${metricText}</span>
        </div>
        <div class="meta">
          ${metaText}
        </div>
      `;
      card.addEventListener("click", () => {
        if (activeCard) {
          activeCard.classList.remove("active");
        }
        activeCard = card;
        card.classList.add("active");
        onSelect(feature);
      });
      list.appendChild(card);
    }
  }

  searchBox.addEventListener("input", () => render(searchBox.value));
  render();
}

function addFeatureToLocalMap(feature, layers) {
  const color = rgbString(feature.class_color_rgb || [100, 100, 100]);
  const centroid = feature.centroid_map_xyz || [null, null, null];

  if (feature.feature_kind === "powerline") {
    const latLngs = getLocalPolylineLatLngs(feature);
    if (latLngs.length >= 2) {
      const line = L.polyline(latLngs, {
        color,
        weight: 4,
        opacity: 0.9
      })
        .bindPopup(buildPopupHtml(feature))
        .addTo(layers.features);

      const labelAnchor = latLngs[Math.floor(latLngs.length / 2)];
      const label = L.marker(labelAnchor, {
        interactive: false,
        icon: L.divIcon({
          className: "",
          html: `<div class="map-label">${escapeHtml(featureDisplayName(feature))}</div>`,
          iconAnchor: [-8, 14]
        })
      }).addTo(layers.labels);

      feature.__leafletLayer = line;
      feature.__label = label;
      feature.__focusBounds = line.getBounds();
      feature.__popupLatLng = labelAnchor;
      return;
    }
  }

  if (!isFiniteNumber(centroid[0]) || !isFiniteNumber(centroid[1])) {
    return;
  }

  const latLng = L.latLng(centroid[1], centroid[0]);
  const marker = L.circleMarker(latLng, {
    radius: 9,
    weight: 2,
    color,
    fillColor: color,
    fillOpacity: 0.8
  })
    .bindPopup(buildPopupHtml(feature))
    .addTo(layers.features);

  const label = L.marker(latLng, {
    interactive: false,
    icon: L.divIcon({
      className: "",
      html: `<div class="map-label">${escapeHtml(featureDisplayName(feature))}</div>`,
      iconAnchor: [-8, 14]
    })
  }).addTo(layers.labels);

  feature.__leafletLayer = marker;
  feature.__label = label;
  feature.__focusBounds = L.latLngBounds([latLng]);
  feature.__popupLatLng = latLng;
}

function addFeatureToGpsMap(feature, map) {
  const color = rgbString(feature.class_color_rgb || [100, 100, 100]);

  if (feature.feature_kind === "powerline") {
    const latLngs = getGpsPolylineLatLngs(feature);
    if (latLngs.length >= 2) {
      const line = L.polyline(latLngs, {
        color,
        weight: 4,
        opacity: 0.9
      })
        .bindPopup(buildPopupHtml(feature))
        .addTo(map);

      const labelAnchor = latLngs[Math.floor(latLngs.length / 2)];
      const label = L.marker(labelAnchor, {
        interactive: false,
        icon: L.divIcon({
          className: "",
          html: `<div class="map-label">${escapeHtml(featureDisplayName(feature))}</div>`,
          iconAnchor: [-8, 14]
        })
      }).addTo(map);

      feature.__leafletLayer = line;
      feature.__label = label;
      feature.__focusBounds = line.getBounds();
      feature.__popupLatLng = labelAnchor;
      return;
    }
  }

  if (!hasGpsPoint(feature)) {
    return;
  }

  const latLng = L.latLng(feature.gps.lat, feature.gps.lon);
  const marker = L.circleMarker(latLng, {
    radius: 9,
    weight: 2,
    color,
    fillColor: color,
    fillOpacity: 0.8
  })
    .bindPopup(buildPopupHtml(feature))
    .addTo(map);

  const label = L.marker(latLng, {
    interactive: false,
    icon: L.divIcon({
      className: "",
      html: `<div class="map-label">${escapeHtml(featureDisplayName(feature))}</div>`,
      iconAnchor: [-8, 14]
    })
  }).addTo(map);

  feature.__leafletLayer = marker;
  feature.__label = label;
  feature.__focusBounds = L.latLngBounds([latLng]);
  feature.__popupLatLng = latLng;
}

function setupLocalMap(features) {
  const map = L.map("map", {
    crs: L.CRS.Simple,
    zoomControl: true,
    preferCanvas: true
  });

  document.getElementById("map-mode").textContent = "Local XY";
  const layers = {
    features: L.layerGroup().addTo(map),
    labels: L.layerGroup().addTo(map)
  };

  const boundsList = [];
  for (const feature of features) {
    addFeatureToLocalMap(feature, layers);
    if (feature.__focusBounds && feature.__focusBounds.isValid()) {
      boundsList.push(feature.__focusBounds);
    }
  }

  if (boundsList.length > 0) {
    let bounds = boundsList[0];
    for (let index = 1; index < boundsList.length; index += 1) {
      bounds = bounds.extend(boundsList[index]);
    }
    map.fitBounds(bounds.pad(0.2));
  } else {
    map.setView([0, 0], 0);
  }

  map.getContainer().appendChild(
    createStatusBanner(
      "GPS is not available for all loaded layers, so this viewer is using local coordinates. Fusion objects and powerlines will only align if they share the same local frame."
    )
  );

  return map;
}

function setupGpsMap(features) {
  const map = L.map("map", {
    zoomControl: true,
    preferCanvas: true
  });
  document.getElementById("map-mode").textContent = "GPS / OSM";

  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 21,
    attribution: "&copy; OpenStreetMap contributors"
  }).addTo(map);

  const boundsList = [];
  for (const feature of features) {
    addFeatureToGpsMap(feature, map);
    if (feature.__focusBounds && feature.__focusBounds.isValid()) {
      boundsList.push(feature.__focusBounds);
    }
  }

  if (boundsList.length > 0) {
    let bounds = boundsList[0];
    for (let index = 1; index < boundsList.length; index += 1) {
      bounds = bounds.extend(boundsList[index]);
    }
    map.fitBounds(bounds.pad(0.25));
  } else {
    map.setView([0, 0], 2);
  }

  return map;
}

function selectFeature(map, feature) {
  if (feature.__focusBounds && feature.__focusBounds.isValid() && feature.__focusBounds.getNorthEast) {
    if (feature.__focusBounds.getSouthWest().equals(feature.__focusBounds.getNorthEast())) {
      map.flyTo(feature.__popupLatLng, Math.max(map.getZoom(), 18), {
        animate: true,
        duration: 0.75
      });
    } else {
      map.fitBounds(feature.__focusBounds.pad(0.15), { animate: true, duration: 0.75 });
    }
  }
  if (feature.__leafletLayer && typeof feature.__leafletLayer.openPopup === "function") {
    feature.__leafletLayer.openPopup(feature.__popupLatLng);
  }
}

async function main() {
  try {
    const { objectPayload, powerlinePayload } = await loadMapData();
    const objectFeatures = (objectPayload.objects || []).map(normalizeObjectFeature);
    const powerlineFeatures = powerlinePayload
      ? (powerlinePayload.powerlines || []).map(normalizePowerlineFeature)
      : [];
    const features = [...objectFeatures, ...powerlineFeatures];

    document.getElementById("object-count").textContent = String(objectFeatures.length);
    document.getElementById("powerline-count").textContent = String(powerlineFeatures.length);

    createLegend(features);

    const gpsMode = features.some(
      (feature) => hasGpsPoint(feature) || getGpsPolylineLatLngs(feature).length >= 2
    );
    const map = gpsMode ? setupGpsMap(features) : setupLocalMap(features);
    createFeatureList(features, (feature) => selectFeature(map, feature));
  } catch (error) {
    document.getElementById("map-mode").textContent = "Error";
    document.getElementById("object-list").innerHTML = `<div class="object-card">${escapeHtml(error.message)}</div>`;
    console.error(error);
  }
}

main();
