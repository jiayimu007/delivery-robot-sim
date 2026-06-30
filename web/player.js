/*
 * Garden Tending Robot - browser player (animation only).
 *
 * This file contains NO algorithm. It does not plan routes (no Dijkstra, no
 * heap, no graph search), it does not steer or integrate any motion model, and
 * it computes no geometry of its own. It only:
 *
 *   1. fetches the JSON the Python engine exported (garden graph + per-frame
 *      trajectories), and
 *   2. draws those precomputed frames on a <canvas>, with a plant picker,
 *      play/pause, a speed slider and a status line.
 *
 * The robot's path and pose come entirely from the engine's recorded frames.
 * Between two recorded frames the player linearly blends position and angle so
 * playback looks smooth -- that is interpolation for drawing, not simulation.
 * To change what the robot does, change the Python engine and re-export; this
 * file just replays whatever it was handed.
 *
 * Data is produced by:  python -m delivery_robot.export web/data
 */

(function () {
  "use strict";

  // ---- colours: mirror the engine's renderer so the page matches the GIF ----
  var COLOR = {
    bg: "#f3efe6",
    road: "#cfd6dd",
    roadEdge: "#b7c0c9",
    route: "#1f6feb",
    node: "#9aa6b2",
    depot: "#0f766e",
    place: "#b4530a",
    placeActive: "#ea580c",
    car: "#1f6feb",
    carEdge: "#0b3d91",
    parcel: "#f59e0b",
    parcelEdge: "#92400e",
    nose: "#fde047",
    label: "#1b2330",
    white: "#ffffff"
  };

  var ROAD_WIDTH = 15;
  var CAR_LENGTH = 26;
  var CAR_WIDTH = 16;

  // ---- UI strings for the EN / 中文 toggle. In 中文 mode the whole UI switches,
  //      including the plant/station names drawn on the canvas (translated via
  //      PLACE_ZH below). The exported data itself stays English. ----
  var I18N = {
    en: {
      title: "Garden Tending Robot",
      subtitle: "A small JS player replaying the watering/fertilizing routes the Python engine planned.",
      destination: "Plant",
      play: "Play",
      pause: "Pause",
      restart: "Restart",
      speed: "Speed",
      loading: "Loading the engine's data...",
      loadError: "Could not load the data. Run: python -m delivery_robot.export web/data",
      atDepot: "At the supply station. Pick a plant.",
      carryingWater: "Carrying water to the ",
      carryingFertilizer: "Carrying fertilizer to the ",
      watered: "Watered the ",
      fertilized: "Fertilized the ",
      returningSuffix: " - returning to the supply station.",
      done: "Back at the supply station. Ready for the next job.",
      note: "The engine (Python, with its tests) computes everything. This page is a small canvas player: it loads the exported routes and animates them. It does not re-plan or re-simulate.",
      photoCaption: "From an actual competition run",
      videoTitle: "Project video",
      langButton: "中文"
    },
    zh: {
      title: "花园养护机器人",
      subtitle: "一个小小的 JS 播放器，回放 Python 引擎规划出的灌溉/施肥路线。",
      destination: "植物",
      play: "播放",
      pause: "暂停",
      restart: "重播",
      speed: "速度",
      loading: "正在加载引擎导出的数据……",
      loadError: "数据加载失败。请运行：python -m delivery_robot.export web/data",
      atDepot: "在供给站待命，请选择一株植物。",
      carryingWater: "正在把水送往 ",
      carryingFertilizer: "正在把肥料送往 ",
      watered: "已灌溉 ",
      fertilized: "已施肥 ",
      returningSuffix: "，正在返回供给站。",
      done: "已回到供给站，准备接下一次养护。",
      note: "所有计算都由引擎（Python，附带测试）完成。本页只是一个小小的画布播放器：它加载导出的路线并播放出来，不重新规划，也不重新仿真。",
      photoCaption: "比赛实际运行中",
      videoTitle: "项目视频",
      langButton: "EN"
    }
  };

  // Scene-label translations for 中文 mode (the abstract garden's generic plant
  // names). Pure localisation; the exported data stays English, keyed by place.
  var PLACE_ZH = {
    depot: "补给站",
    roses: "玫瑰",
    tomatoes: "番茄",
    herbs: "香草",
    lavender: "薰衣草",
    apple_tree: "苹果树"
  };

  var lang = "en";

  // ---- DOM ----
  var canvas = document.getElementById("view");
  var ctx = canvas.getContext("2d");
  var destSel = document.getElementById("dest");
  var playBtn = document.getElementById("play");
  var restartBtn = document.getElementById("restart");
  var speedInput = document.getElementById("speed");
  var speedOut = document.getElementById("speedOut");
  var statusEl = document.getElementById("status");
  var langBtn = document.getElementById("lang");

  // ---- state ----
  var town = null;            // the exported town graph
  var route = null;           // the exported route for the current destination
  var routeCache = {};        // place -> loaded route JSON
  var margin = 11;            // canvas inset, from the export

  var playing = false;
  var frameF = 0;             // fractional frame index (for smooth playback)
  var lastTs = 0;
  var BASE_FPS = 60;          // recorded-frames-per-second at the 1.0x setting
  var rafId = 0;

  function t(key) {
    return (I18N[lang] && I18N[lang][key] != null) ? I18N[lang][key] : I18N.en[key];
  }

  // A place's display name: translated in 中文 mode, English otherwise.
  function placeName(placeKey, englishLabel) {
    if (lang === "zh" && PLACE_ZH[placeKey]) return PLACE_ZH[placeKey];
    return englishLabel || placeKey;
  }

  // ---------------------------------------------------------------- drawing --
  function toCanvas(x, y) {
    return [x + margin, y + margin];
  }

  function clear() {
    ctx.fillStyle = COLOR.bg;
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }

  function drawRoads() {
    var i, a, b, p, q, n, cx, cy;
    // casing, then the lighter surface on top
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.strokeStyle = COLOR.roadEdge;
    ctx.lineWidth = ROAD_WIDTH + 4;
    for (i = 0; i < town.edges.length; i++) {
      a = town.nodes[town.edges[i][0]];
      b = town.nodes[town.edges[i][1]];
      p = toCanvas(a.x, a.y);
      q = toCanvas(b.x, b.y);
      ctx.beginPath();
      ctx.moveTo(p[0], p[1]);
      ctx.lineTo(q[0], q[1]);
      ctx.stroke();
    }
    ctx.strokeStyle = COLOR.road;
    ctx.lineWidth = ROAD_WIDTH;
    for (i = 0; i < town.edges.length; i++) {
      a = town.nodes[town.edges[i][0]];
      b = town.nodes[town.edges[i][1]];
      p = toCanvas(a.x, a.y);
      q = toCanvas(b.x, b.y);
      ctx.beginPath();
      ctx.moveTo(p[0], p[1]);
      ctx.lineTo(q[0], q[1]);
      ctx.stroke();
    }
    // round junction caps so corners look filled
    for (n = 0; n < town.nodes.length; n++) {
      cx = town.nodes[n].x + margin;
      cy = town.nodes[n].y + margin;
      ctx.fillStyle = COLOR.roadEdge;
      ctx.beginPath();
      ctx.arc(cx, cy, (ROAD_WIDTH + 4) / 2, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = COLOR.road;
      ctx.beginPath();
      ctx.arc(cx, cy, ROAD_WIDTH / 2, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Draw the planned route the engine returned, as a dashed blue polyline. The
  // node-id list comes from the export (Dijkstra's output); the player only
  // connects those points, it does not choose them.
  function drawPlannedRoute(path, carrying) {
    if (!path || path.length < 2) return;
    ctx.save();
    ctx.strokeStyle = COLOR.route;
    ctx.lineWidth = 5;
    ctx.lineCap = "butt";
    ctx.globalAlpha = carrying ? 1 : 0.45;
    ctx.setLineDash([2, 7]);
    ctx.beginPath();
    var p0 = toCanvas(town.nodes[path[0]].x, town.nodes[path[0]].y);
    ctx.moveTo(p0[0], p0[1]);
    for (var i = 1; i < path.length; i++) {
      var p = toCanvas(town.nodes[path[i]].x, town.nodes[path[i]].y);
      ctx.lineTo(p[0], p[1]);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawNodes() {
    for (var i = 0; i < town.nodes.length; i++) {
      var n = town.nodes[i];
      if (n.place) continue;
      var cx = n.x + margin, cy = n.y + margin;
      ctx.fillStyle = COLOR.node;
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  function drawPlaces(activeId) {
    ctx.font = "13px -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif";
    ctx.textBaseline = "alphabetic";
    for (var i = 0; i < town.nodes.length; i++) {
      var n = town.nodes[i];
      if (!n.place) continue;
      var cx = n.x + margin, cy = n.y + margin;
      var isDepot = n.place === "depot";
      var isActive = activeId === n.id;
      var r = isDepot ? 11 : 9;
      ctx.fillStyle = isDepot ? COLOR.depot : (isActive ? COLOR.placeActive : COLOR.place);
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = COLOR.white;
      ctx.stroke();
      // Plant name or "Supply Station": English from the export, translated for
      // display in 中文 mode only.
      var label = placeName(n.place, n.label);
      var tw = ctx.measureText(label).width;
      var ly = (n.y > town.height - 40) ? cy - 18 : cy + 22;
      ctx.fillStyle = COLOR.label;
      ctx.fillText(label, cx - tw / 2, ly);
    }
  }

  function drawCar(pose) {
    var cx = pose.x + margin, cy = pose.y + margin;
    var L = CAR_LENGTH, W = CAR_WIDTH;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(pose.theta);
    // body
    ctx.fillStyle = COLOR.car;
    ctx.strokeStyle = COLOR.carEdge;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(-L / 2, -W / 2, L, W);
    ctx.fill();
    ctx.stroke();
    // nose triangle (heading)
    ctx.fillStyle = COLOR.nose;
    ctx.beginPath();
    ctx.moveTo(L / 2, 0);
    ctx.lineTo(L / 2 - 7, -5);
    ctx.lineTo(L / 2 - 7, 5);
    ctx.closePath();
    ctx.fill();
    // the water/fertilizer unit (a small block) while carrying
    if (pose.carrying) {
      ctx.fillStyle = COLOR.parcel;
      ctx.strokeStyle = COLOR.parcelEdge;
      ctx.beginPath();
      ctx.rect(-6, -6, 12, 12);
      ctx.fill();
      ctx.stroke();
    }
    ctx.restore();
  }

  // Blend two recorded poses for smooth playback. Pure rendering helper:
  // linear on position, shortest-arc linear on angle. No physics.
  function blendPose(a, b, f) {
    var dth = b.theta - a.theta;
    while (dth > Math.PI) dth -= Math.PI * 2;
    while (dth < -Math.PI) dth += Math.PI * 2;
    return {
      x: a.x + (b.x - a.x) * f,
      y: a.y + (b.y - a.y) * f,
      theta: a.theta + dth * f,
      carrying: a.carrying
    };
  }

  function currentPose() {
    var frames = route.frames;
    var i = Math.floor(frameF);
    if (i >= frames.length - 1) {
      return frames[frames.length - 1];
    }
    return blendPose(frames[i], frames[i + 1], frameF - i);
  }

  // Which planned leg to highlight right now: outbound while carrying, return
  // after the drop. Both lists come straight from the export.
  function activePath(carrying) {
    return carrying ? route.pathToDest : route.pathToDepot;
  }

  function statusFor(pose, atEnd) {
    if (atEnd) return t("done");
    var label = placeName(route.place, route.label);
    var isWater = route.cargo === "water";
    if (pose.carrying) {
      return (isWater ? t("carryingWater") : t("carryingFertilizer")) + label;
    }
    return (isWater ? t("watered") : t("fertilized")) + label + t("returningSuffix");
  }

  function render() {
    clear();
    drawRoads();
    if (route) {
      var pose = currentPose();
      var atEnd = frameF >= route.frames.length - 1;
      drawPlannedRoute(activePath(pose.carrying), pose.carrying);
      drawNodes();
      drawPlaces(route.destId);
      drawCar(pose);
      statusEl.textContent = statusFor(pose, atEnd && !playing);
    } else {
      drawNodes();
      drawPlaces(null);
    }
  }

  // ------------------------------------------------------------- animation --
  function tick(ts) {
    if (!playing) return;
    if (!lastTs) lastTs = ts;
    var dt = (ts - lastTs) / 1000;
    lastTs = ts;

    var speed = parseFloat(speedInput.value);
    frameF += dt * BASE_FPS * speed;

    if (frameF >= route.frames.length - 1) {
      frameF = route.frames.length - 1;
      render();
      stop();
      return;
    }
    render();
    rafId = requestAnimationFrame(tick);
  }

  function start() {
    if (!route) return;
    if (frameF >= route.frames.length - 1) frameF = 0; // replay from the top
    playing = true;
    lastTs = 0;
    playBtn.textContent = t("pause");
    rafId = requestAnimationFrame(tick);
  }

  function stop() {
    playing = false;
    cancelAnimationFrame(rafId);
    playBtn.textContent = t("play");
    render();
  }

  function restart() {
    frameF = 0;
    if (playing) {
      lastTs = 0;
    } else {
      render();
    }
  }

  // ------------------------------------------------------------- data load --
  function fetchJSON(url) {
    return fetch(url, { cache: "no-cache" }).then(function (r) {
      if (!r.ok) throw new Error("HTTP " + r.status + " for " + url);
      return r.json();
    });
  }

  function loadRoute(place) {
    if (routeCache[place]) {
      return Promise.resolve(routeCache[place]);
    }
    return fetchJSON("data/routes/" + place + ".json").then(function (r) {
      routeCache[place] = r;
      return r;
    });
  }

  function selectDestination(place) {
    stop();
    statusEl.textContent = t("loading");
    loadRoute(place).then(function (r) {
      route = r;
      frameF = 0;
      render();
      start();
    }).catch(function (err) {
      statusEl.textContent = t("loadError");
      // surface the detail in the console for whoever is debugging the export
      if (window.console) console.error(err);
    });
  }

  // --------------------------------------------------------------- chrome ---
  function applyLanguage() {
    var d = I18N[lang];
    var nodes = document.querySelectorAll("[data-i18n]");
    for (var i = 0; i < nodes.length; i++) {
      var key = nodes[i].getAttribute("data-i18n");
      if (d[key] != null) nodes[i].textContent = d[key];
    }
    document.documentElement.lang = (lang === "zh") ? "zh-CN" : "en";
    langBtn.textContent = t("langButton");
    playBtn.textContent = playing ? t("pause") : t("play");
    if (town) {                       // re-label the plant dropdown, keep the pick
      var keep = destSel.value;
      fillDestinations();
      destSel.value = keep;
    }
    render(); // refresh the status line + scene labels in the new language
  }

  // Look up a place key's English display label from the exported nodes.
  function labelOf(place) {
    for (var i = 0; i < town.nodes.length; i++) {
      if (town.nodes[i].place === place && town.nodes[i].label) {
        return town.nodes[i].label;
      }
    }
    return place;
  }

  function fillDestinations() {
    destSel.innerHTML = "";
    for (var i = 0; i < town.destinations.length; i++) {
      var place = town.destinations[i];
      var opt = document.createElement("option");
      opt.value = place;
      opt.textContent = placeName(place, labelOf(place)); // translated in 中文 mode
      destSel.appendChild(opt);
    }
  }

  function updateSpeedLabel() {
    speedOut.textContent = parseFloat(speedInput.value).toFixed(2).replace(/0$/, "") + "x";
  }

  // ----------------------------------------------------------------- wire ---
  playBtn.addEventListener("click", function () {
    if (playing) stop(); else start();
  });
  restartBtn.addEventListener("click", restart);
  speedInput.addEventListener("input", updateSpeedLabel);
  destSel.addEventListener("change", function () {
    selectDestination(destSel.value);
  });
  langBtn.addEventListener("click", function () {
    lang = (lang === "en") ? "zh" : "en";
    applyLanguage();
  });

  updateSpeedLabel();

  // Load the town first, then the initial route.
  fetchJSON("data/town.json").then(function (data) {
    town = data;
    margin = town.margin != null ? town.margin : 11;
    canvas.width = town.width + 2 * margin;
    canvas.height = town.height + 2 * margin;
    fillDestinations();
    render();
    if (town.destinations.length) {
      destSel.value = town.destinations[0];
      selectDestination(town.destinations[0]);
    }
  }).catch(function (err) {
    statusEl.textContent = t("loadError");
    if (window.console) console.error(err);
  });
})();
