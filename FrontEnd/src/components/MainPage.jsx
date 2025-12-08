import React, { useRef, useEffect, useState, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import Sidebar from "./SideBar";
import JoyStick from "./JoyStick";
import {
  FaBatteryHalf,
  FaPlay,
  FaPause,
  FaExpand,
  FaCompress,
  FaTrash,
  FaMicrophone,
  FaPaperPlane,
  FaLock,
  FaUnlock,
  FaSignOutAlt,
  FaPlus,
  FaEdit,
} from "react-icons/fa";
import { toast } from "react-hot-toast";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../utils/auth";
import {
  connectBridgeSocket,
  fetchBridgeStatus,
  sendCmdVel,
} from "../utils/amrBridge";
import "./MainPage.css";

const API_V1_BASE = `${API_BASE}/api/v1`;
const DEFAULT_CAMERA_URL =
  process.env.REACT_APP_CAMERA_URL ||
  `${process.env.REACT_APP_AMR_BRIDGE_BASE || "http://localhost:8000"}/api/camera/stream`;

const FALLBACK_STATS = {
  overview: {
    totalKm: 182.4,
    missionsCompleted: 47,
    avgSpeed: 1.8,
    operatingHours: 326,
    deltaKm: 4.3,
    missionSuccessRate: 98,
  },
  missionTrend: [
    { label: "Mon", completed: 5, incidents: 0 },
    { label: "Tue", completed: 7, incidents: 1 },
    { label: "Wed", completed: 6, incidents: 0 },
    { label: "Thu", completed: 8, incidents: 1 },
    { label: "Fri", completed: 9, incidents: 0 },
  ],
  monthlyMovement: [
    { month: "Jan", km: 118 },
    { month: "Feb", km: 142 },
    { month: "Mar", km: 131 },
    { month: "Apr", km: 155 },
    { month: "May", km: 162 },
    { month: "Jun", km: 174 },
  ],
  batterySeries: [
    { time: "08:00", voltage: 48.2, power: 182 },
    { time: "09:00", voltage: 47.8, power: 176 },
    { time: "10:00", voltage: 47.4, power: 171 },
    { time: "11:00", voltage: 46.9, power: 168 },
    { time: "12:00", voltage: 47.1, power: 170 },
    { time: "13:00", voltage: 46.7, power: 166 },
    { time: "14:00", voltage: 46.3, power: 164 },
  ],
  batteryStatus: {
    packVoltage: 46.9,
    packCurrent: 38.2,
    stateOfCharge: 78,
    temperature: "32°C",
    cycles: 412,
    health: "Good",
    cells: [
      { id: "Cell A", voltage: 3.9 },
      { id: "Cell B", voltage: 3.89 },
      { id: "Cell C", voltage: 3.88 },
      { id: "Cell D", voltage: 3.87 },
    ],
  },
  turns: { left: 132, right: 148 },
};

const MainPage = () => {
  const navigate = useNavigate();
  const [emergencyClicked, setEmergencyClicked] = useState(false);
  const mapRef = useRef(null);
  const layoutRef = useRef(null);
  const chatContainerRef = useRef(null);
  const bridgeBase = process.env.REACT_APP_AMR_BRIDGE_BASE || "http://localhost:8000";

  // Chat state
  const [chatMessages, setChatMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your robot assistant. How can I help you today?",
      sender: "robot",
      timestamp: new Date().toISOString(),
      status: "Delivered",
    },
  ]);
  const [chatInput, setChatInput] = useState("");
  const [isRecording, setIsRecording] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const chatQuickPrompts = [
    "Provide current mission status",
    "Return to docking station",
    "Begin perimeter scan",
    "Share latest sensor alerts",
    "Start recording telemetry for 5 minutes",
  ];

  useEffect(() => {
    if (chatContainerRef.current) {
      chatContainerRef.current.scrollTop =
        chatContainerRef.current.scrollHeight;
    }
  }, [chatMessages]);

  // new: zoom state for map
  const [zoomLevel, setZoomLevel] = useState(0); // 0 .. 0.3
  const toggleMapZoom = useCallback(() => {
    setZoomLevel((z) => (z === 0 ? 0.3 : 0));
  }, []);
  const zoomIn = useCallback(
    (e) => {
      e && e.stopPropagation();
      setZoomLevel((z) => Math.min(0.3, +(z + 0.1).toFixed(2)));
    },
    [setZoomLevel],
  );
  const zoomOut = useCallback(
    (e) => {
      e && e.stopPropagation();
      setZoomLevel((z) => Math.max(0, +(z - 0.1).toFixed(2)));
    },
    [setZoomLevel],
  );
  // minimize main center (map) and joystick
  const [minimizedMain, setMinimizedMain] = useState(false);

  // joystick state (last vector from physical joystick or keyboard WASD)
  const [joystickState, setJoystickState] = useState({
    x: 0,
    y: 0,
    force: 0,
    angle: 0,
    type: null,
  });
  const [isLocked, setIsLocked] = useState(false);
  const LOCK_TOAST_ID = "lock-toast";

  const showLockedAttemptToast = useCallback(() => {
    // single toast id to avoid duplicates when user repeatedly clicks
    toast.dismiss("locked-attempt");
    toast.error("The screen is locked", { id: "locked-attempt" });
  }, []);

  const handleLogout = useCallback(() => {
    // clear tokens and go back to login
    clearAuthTokens();
    navigate("/");
  }, [navigate]);

  const handleToggleLock = useCallback(() => {
    setIsLocked((prev) => {
      const next = !prev;
      // ensure only one toast shown by using a fixed id and dismissing previous
      toast.dismiss(LOCK_TOAST_ID);
      if (next) {
        toast.error("Console locked", { id: LOCK_TOAST_ID });
      } else {
        toast.success("Console unlocked", { id: LOCK_TOAST_ID });
      }
      return next;
    });
  }, []);
  const [bridgeStatus, setBridgeStatus] = useState({
    connected: false,
    latest: null,
    lastUpdated: null,
    error: "",
    endpoint: bridgeBase,
  });
  const bridgeConnRef = useRef({ send: null, close: null });
  const bridgeConnectedRef = useRef(false);
  const lastCmdSentRef = useRef(0);
  const reconnectRef = useRef(null);

  // handler passed to JoyStick so component receives events (stick or keyboard)
  const handleJoystickMove = useCallback((data) => {
    // data shape: { x, y, force, angle, ... }
    setJoystickState({
      x: data.x ?? 0,
      y: data.y ?? 0,
      force: data.force ?? 0,
      angle: data.angle ?? 0,
      type: data.type || "joystick",
    });
    // TODO: translate to robot commands / send to backend if needed
    // console.debug("Joystick move:", data);
  }, []);

  const [batteryLevel, setBatteryLevel] = useState(100);
  useEffect(() => {
    let batteryObj = null;
    let mounted = true;
    const updateLevel = () => {
      if (!batteryObj) return;
      const lvl = Math.round(batteryObj.level * 100);
      if (mounted) setBatteryLevel(lvl);
    };
    if (navigator.getBattery) {
      navigator.getBattery().then((b) => {
        batteryObj = b;
        updateLevel();
        batteryObj.addEventListener("levelchange", updateLevel);
      });
    } else {
      // fallback or simulated value
      setBatteryLevel(100);
    }
    return () => {
      mounted = false;
      if (batteryObj && batteryObj.removeEventListener) {
        batteryObj.removeEventListener("levelchange", updateLevel);
      }
    };
  }, []);

  // Connect to AMR bridge (FastAPI/ROS2) via WebSocket with HTTP poll fallback
  const startBridgeConnection = useCallback(() => {
    // Close previous socket if any
    bridgeConnRef.current?.close?.();

    const conn = connectBridgeSocket({
      baseUrl: bridgeBase,
      onOpen: () => {
        bridgeConnectedRef.current = true;
        setBridgeStatus((prev) => ({ ...prev, connected: true, error: "", endpoint: bridgeBase }));
      },
      onStatus: (data) => {
        bridgeConnectedRef.current = true;
        setBridgeStatus({
          connected: true,
          latest: data || {},
          lastUpdated: Date.now(),
          error: "",
          endpoint: bridgeBase,
        });
      },
      onError: (err) => {
        setBridgeStatus((prev) => ({
          ...prev,
          error: err?.message || "Bridge WebSocket error",
          endpoint: bridgeBase,
        }));
      },
      onClose: () => {
        bridgeConnectedRef.current = false;
        setBridgeStatus((prev) => ({ ...prev, connected: false, endpoint: bridgeBase }));
      },
    });

    bridgeConnRef.current = conn;
  }, [bridgeBase]);

  useEffect(() => {
    startBridgeConnection();

    const pollInterval = setInterval(async () => {
      if (bridgeConnectedRef.current) return;
      try {
        const status = await fetchBridgeStatus(bridgeBase);
        setBridgeStatus((prev) => ({
          ...prev,
          connected: status?.status === "online",
          latest: status?.latest_status || prev.latest,
          lastUpdated: Date.now(),
          error: "",
          endpoint: bridgeBase,
        }));
      } catch (err) {
        setBridgeStatus((prev) => ({
          ...prev,
          error: err?.message || "Bridge status fetch failed",
          endpoint: bridgeBase,
        }));
        // auto retry after short backoff to avoid sticky offline state
        if (!reconnectRef.current) {
          reconnectRef.current = setTimeout(() => {
            reconnectRef.current = null;
            startBridgeConnection();
          }, 4000);
        }
      }
    }, 5000);

    return () => {
      bridgeConnRef.current?.close?.();
      clearInterval(pollInterval);
      if (reconnectRef.current) clearTimeout(reconnectRef.current);
    };
  }, [startBridgeConnection, bridgeBase]);

  // Send joystick vectors as cmd_vel (throttled) to the bridge
  useEffect(() => {
    if (isLocked || emergencyClicked) return;

    const now = Date.now();
    if (now - lastCmdSentRef.current < 120) return;
    lastCmdSentRef.current = now;

    const linear_x = Number((joystickState.y * 0.5).toFixed(3)); // forward/back
    const angular_z = Number((joystickState.x * 1.0).toFixed(3)); // rotate
    const payload = { linear_x, linear_y: 0, angular_z };

    const sentViaSocket =
      bridgeConnRef.current?.send &&
      bridgeConnRef.current.send({ type: "cmd_vel", data: payload });

    if (!sentViaSocket) {
      sendCmdVel(payload).catch((err) => {
        setBridgeStatus((prev) => ({
          ...prev,
          error: err?.message || "cmd_vel request failed",
        }));
      });
    }
  }, [joystickState, isLocked, emergencyClicked]);

  // When locking UI or hitting emergency stop, send a zero-velocity command
  useEffect(() => {
    if (!isLocked && !emergencyClicked) return;
    const payload = { linear_x: 0, linear_y: 0, angular_z: 0 };
    const sentViaSocket =
      bridgeConnRef.current?.send &&
      bridgeConnRef.current.send({ type: "cmd_vel", data: payload });
    if (!sentViaSocket) {
      sendCmdVel(payload).catch(() => {});
    }
  }, [isLocked, emergencyClicked]);

  // fullscreen state and listener
  const [isFullScreen, setIsFullScreen] = useState(false);
  useEffect(() => {
    const onFsChange = () => {
      const fsElement =
        document.fullscreenElement || document.webkitFullscreenElement;
      setIsFullScreen(fsElement === layoutRef.current);
    };
    document.addEventListener("fullscreenchange", onFsChange);
    document.addEventListener("webkitfullscreenchange", onFsChange);
    return () => {
      document.removeEventListener("fullscreenchange", onFsChange);
      document.removeEventListener("webkitfullscreenchange", onFsChange);
    };
  }, []);

  // toggleFullScreen function
  const toggleFullScreen = async () => {
    try {
      const target = layoutRef.current;
      if (!target) return;
      const fsElement =
        document.fullscreenElement || document.webkitFullscreenElement;
      if (!fsElement) {
        if (target.requestFullscreen) {
          await target.requestFullscreen();
        } else if (target.webkitRequestFullscreen) {
          target.webkitRequestFullscreen(); // Safari
        }
      } else {
        if (document.exitFullscreen) {
          await document.exitFullscreen();
        } else if (document.webkitExitFullscreen) {
          document.webkitExitFullscreen();
        }
      }
    } catch (e) {
      console.warn("Fullscreen error:", e);
    }
  };

  // dropdown & play/pause state for header button
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);

  // new: right-side page (renders when a widen icon is clicked)
  const [rightPage, setRightPage] = useState(null);

  // sample maps data — replace image paths with your real map images
  const initialMaps = [
    {
      id: "cfl_gf",
      name: "CFL_GF",
      createdBy: "CNDE IITM",
      // removed local screenshot reference
      image: "",
      status: "Active",
    },
    {
      id: "shobot_arena",
      name: "shobot_arena",
      createdBy: "ANSCER ADMIN",
      image: "/images/maps/shobot_arena.png",
      status: "",
    },
    {
      id: "shobot_arena2",
      name: "shobot_arena2",
      createdBy: "ANSCER ADMIN",
      image: "/images/maps/shobot_arena2.png",
      status: "",
    },
    /* zones entry: cleared local screenshot */
    {
      id: "zones",
      name: "Zones",
      createdBy: "ANSCER ADMIN",
      image: "",
      status: "",
    },
    /* waypoints entry: cleared local screenshot */
    {
      id: "waypoints",
      name: "Waypoints",
      createdBy: "ANSCER ADMIN",
      image: "",
      status: "",
    },
    /* users preview cleared */
    {
      id: "users",
      name: "Users",
      createdBy: "ANSCER ADMIN",
      image: "",
      status: "",
    },
  ];
  const [mapsList, setMapsList] = useState(initialMaps);
  const [selectedMap, setSelectedMap] = useState(initialMaps[0]);
  // unified map search (single dropdown selects field, single input is the query)
  const [mapSearchField, setMapSearchField] = useState("any"); // any,name,createdBy,category,createdAt,status
  const [mapSearchTerm, setMapSearchTerm] = useState("");

  // waypoints data + selection
  const initialWaypoints = [
    {
      id: "wp1",
      name: "WP_A",
      category: "Nav",
      active: true,
      geom: "Point(12 34)",
      createdAt: "2025-11-17",
      notes: "First waypoint",
    },
    {
      id: "wp2",
      name: "WP_B",
      category: "Inspect",
      active: false,
      geom: "Point(98 76)",
      createdAt: "2025-11-17",
      notes: "Inspection point",
    },
    {
      id: "wp3",
      name: "WP_C",
      category: "Charge",
      active: true,
      geom: "Point(44 55)",
      createdAt: "2025-11-16",
      notes: "Charging pad",
    },
  ];
  const [waypoints, setWaypoints] = useState(initialWaypoints);
  const [waypointFormOpen, setWaypointFormOpen] = useState(false);
  const [waypointForm, setWaypointForm] = useState({
    name: "",
    category: "Nav",
    geom: "",
    notes: "",
    active: true,
  });
  const [selectedWaypointId, setSelectedWaypointId] = useState(null);

  // missions data + selection (added)
  const initialMissions = [
    {
      id: "m1",
      name: "Inspect Zone A",
      owner: "CNDE",
      status: "Draft",
      createdAt: "2025-11-17",
      notes: "Routine inspection",
    },
    {
      id: "m2",
      name: "Delivery Route 3",
      owner: "ANSCER ADMIN",
      status: "Scheduled",
      createdAt: "2025-11-16",
      notes: "Delivery to docks",
    },
    {
      id: "m3",
      name: "Battery Check",
      owner: "CNDE",
      status: "Completed",
      createdAt: "2025-11-15",
      notes: "Post-run check",
    },
  ];
  const [missions, setMissions] = useState(initialMissions);
  const [selectedMissionId, setSelectedMissionId] = useState(null);
  const [missionActionLoading, setMissionActionLoading] = useState(false);
  const handleSelectMission = (id) => setSelectedMissionId(id);
  const [missionFormOpen, setMissionFormOpen] = useState(false);
  const [missionForm, setMissionForm] = useState({
    name: "",
    owner: "",
    status: "Draft",
    notes: "",
  });
  const [cameraUrl, setCameraUrl] = useState(DEFAULT_CAMERA_URL);
  const bridgeApiLayers = [
    { layer: "Control", purpose: "Teleop, localization, PLC", ids: "9001, 9002, 9003" },
    { layer: "Push", purpose: "Live robot position", ids: "8001" },
    { layer: "Mapping", purpose: "Start/stop mapping, map data", ids: "9102, 9103, 9101" },
    { layer: "Task Manager", purpose: "Missions & PLC IO", ids: "8766, 8767, 8768" },
    { layer: "Status", purpose: "Lidar, odom, path, PLC inputs", ids: "7001–7008" },
    { layer: "WiFi", purpose: "Setup network", ids: "4001–4003" },
    { layer: "Navigation", purpose: "Autonomous motion", ids: "3001–3003" },
  ];

  // users data + selection (fixes eslint no-undef)
  const [users, setUsers] = useState([]);
  const [selectedUserId, setSelectedUserId] = useState(null);
  const [usersLoading, setUsersLoading] = useState(false);
  const [userActionLoading, setUserActionLoading] = useState(false);
  const [usersError, setUsersError] = useState("");

  const initialZones = [
    {
      id: "z1",
      name: "Assembly Lane",
      category: "Safe",
      active: true,
      geometry: "Polygon(32,18…)",
      createdAt: "2025-11-17",
    },
    {
      id: "z2",
      name: "Battery Bay",
      category: "No-Go",
      active: true,
      geometry: "Polygon(27,11…)",
      createdAt: "2025-11-15",
    },
    {
      id: "z3",
      name: "Dock Tunnel",
      category: "Caution",
      active: false,
      geometry: "Line(82,90…)",
      createdAt: "2025-11-14",
    },
  ];
  const [zones, setZones] = useState(initialZones);
  const [zoneFormOpen, setZoneFormOpen] = useState(false);
  const [zoneForm, setZoneForm] = useState({
    name: "",
    category: "Safe",
    geometry: "",
    active: true,
  });

  const [analyticsSummary, setAnalyticsSummary] = useState([
    { label: "Incidents", value: 2, trend: "+1 vs last week" },
    { label: "Stops Issued", value: 14, trend: "-3 vs last week" },
    { label: "Battery Swaps", value: 5, trend: "Stable" },
    { label: "Avg. Cycle", value: "42 min", trend: "±0" },
  ]);

  const [analyticsSeries, setAnalyticsSeries] = useState([
    12, 18, 22, 16, 24, 26, 20,
  ]);
  const [analyticsAlerts, setAnalyticsAlerts] = useState([
    {
      id: "alert1",
      title: "Obstacle spikes",
      detail: "Lidar reported 5 high-density events on Dock Tunnel.",
    },
    {
      id: "alert2",
      title: "Slow return",
      detail: "Mission Delivery Route 3 exceeded SLA by 4 min.",
    },
  ]);

  const [diagnosticsPanels, setDiagnosticsPanels] = useState([
    {
      id: "battery",
      title: "Battery Health",
      value: "93%",
      status: "Nominal",
      detail: "Cells balanced",
    },
    {
      id: "motors",
      title: "Drive Motors",
      value: "Temp 48°C",
      status: "Monitoring",
      detail: "Torque variance +3%",
    },
    {
      id: "sensors",
      title: "Sensor Suite",
      value: "All online",
      status: "Nominal",
      detail: "Last calibration 12h ago",
    },
  ]);

  const [logEvents, setLogEvents] = useState([
    {
      id: "log1",
      ts: "10:42:01",
      system: "Navigation",
      message: "Replanned path around blocked aisle",
      level: "info",
    },
    {
      id: "log2",
      ts: "10:15:22",
      system: "Safety",
      message: "Emergency stop acknowledged",
      level: "warn",
    },
    {
      id: "log3",
      ts: "09:57:10",
      system: "Battery",
      message: "Pack voltage dipped to 45.9V",
      level: "warn",
    },
  ]);
  
  const [missionHistory, setMissionHistory] = useState([
    {
      id: "mh1",
      mission: "Inspect Zone A",
      window: "08:00–08:18",
      outcome: "Completed",
      notes: "No issues",
    },
    {
      id: "mh2",
      mission: "Delivery Route 3",
      window: "08:30–09:10",
      outcome: "Delayed",
      notes: "Obstacle at Dock Tunnel",
    },
    {
      id: "mh3",
      mission: "Battery Check",
      window: "09:15–09:32",
      outcome: "Completed",
      notes: "Pack swap verified",
    },
  ]);

  const [bagFiles, setBagFiles] = useState([
    {
      id: "bag1",
      name: "mission-0915.bag",
      duration: "15m",
      size: "1.4 GB",
      status: "Uploaded",
    },
    {
      id: "bag2",
      name: "mission-1030.bag",
      duration: "26m",
      size: "2.7 GB",
      status: "Processing",
    },
  ]);

  const [robotSettingsState, setRobotSettingsState] = useState({
    autopilot: true,
    safeMode: true,
    remoteDiagnostics: false,
    pathOptimization: true,
  });

  const [accountProfile, setAccountProfile] = useState({
    fullName: "",
    email: "",
    team: "",
    shift: "",
  });
  const [profileSaving, setProfileSaving] = useState(false);
  const [profileError, setProfileError] = useState("");
  const [profileSuccess, setProfileSuccess] = useState("");

  const [selectedTheme, setSelectedTheme] = useState("system");

  const [securityPreferences, setSecurityPreferences] = useState({
    twoFactor: true,
    autoLock: true,
    anomalyAlerts: true,
  });

  const [securityEvents, setSecurityEvents] = useState([
    {
      id: "sec1",
      ts: "09:44",
      actor: "ops-admin",
      action: "API token created",
      context: "Main console",
    },
    {
      id: "sec2",
      ts: "08:12",
      actor: "robot-01",
      action: "Cert renewed",
      context: "Device",
    },
  ]);

  const [integrationItems, setIntegrationItems] = useState([
    {
      id: "rest",
      name: "REST API",
      status: "Connected",
      description: "Push missions from MES",
    },
    {
      id: "slack",
      name: "Slack Bot",
      status: "Disconnected",
      description: "Alerts to #robot-ops",
    },
    {
      id: "grafana",
      name: "Grafana",
      status: "Connected",
      description: "Telemetry dashboards",
    },
  ]);

  const [statsData, setStatsData] = useState(FALLBACK_STATS);
  const [statsError, setStatsError] = useState("");
  const [statsLoading, setStatsLoading] = useState(false);
  const [batteryModalOpen, setBatteryModalOpen] = useState(false);

  const handleSelectWaypoint = (wpId) => {
    setSelectedWaypointId(wpId);
    const wpMap = mapsList.find((m) => m.id === "waypoints");
    if (wpMap) setSelectedMap(wpMap);
  };

  const handleSidebarSelect = (id) => {
    // open right pane for create sub-items and monitor sub-items
    const createIds = new Set([
      "maps",
      "zones",
      "waypoints",
      "missions",
      "users",
    ]);
    const monitorIds = new Set([
      "analytics",
      "diagnostics",
      "logs",
      "mission-logs",
      "robot-bags",
    ]);
    const settingsIds = new Set([
      "robot-settings",
      "account",
      "appearance",
      "security",
      "integrations",
    ]);
    if (id === "chat") {
      setRightPage("chat");
    } else if (id === "stats") {
      setRightPage("stats");
    } else if (createIds.has(id) || monitorIds.has(id) || settingsIds.has(id)) {
      setRightPage(id);
      // if it's the maps "page", preselect first map
      if (id === "maps") {
        setSelectedMap(mapsList[0]);
      } else if (id === "zones") {
        // select the zones entry so left-side panel shows the requested path
        const z = mapsList.find((m) => m.id === "zones");
        if (z) setSelectedMap(z);
      } else if (id === "waypoints") {
        // select the waypoints entry so main area shows the provided screenshot path
        const w = mapsList.find((m) => m.id === "waypoints");
        if (w) setSelectedMap(w);
      } else if (id === "users") {
        // select the users preview so left info shows the screenshot/path
        const u = mapsList.find((m) => m.id === "users");
        if (u) setSelectedMap(u);
      }
    } else {
      // ignore main sidebar icons (do not render a page)
      setRightPage(null);
    }
  };

  // Map modal state for preview / edit / create
  const [mapModalOpen, setMapModalOpen] = useState(false);
  const [mapModalMode, setMapModalMode] = useState("preview"); // "preview" | "edit" | "create"
  const [mapModalMap, setMapModalMap] = useState(null);

  // form used for create/edit
  const [mapForm, setMapForm] = useState({
    id: null,
    name: "",
    createdBy: "",
    image: "",
    status: "",
    category: "",
    createdAt: "",
  });

  const openMapModal = (mode, map = null) => {
    setMapModalMode(mode);
    setMapModalMap(map);
    if (!map) {
      setMapForm({
        id: null,
        name: "",
        createdBy: "",
        image: "",
        status: "",
        category: "",
        createdAt: new Date().toISOString().slice(0, 10),
      });
    } else {
      setMapForm({
        id: map.id,
        name: map.name || "",
        createdBy: map.createdBy || "",
        image: map.image || "",
        status: map.status || "",
        category: map.category || "",
        createdAt: map.createdAt || "",
      });
    }
    setMapModalOpen(true);
  };

  const closeMapModal = () => {
    setMapModalOpen(false);
    setMapModalMap(null);
  };

  const handleMapAction = async (action, map) => {
    try {
      if (action === "edit") {
        openMapModal("edit", map);
        return;
      }
      if (action === "delete") {
        const ok = window.confirm(
          `Delete map "${map?.name || map?.id}"? This cannot be undone.`,
        );
        if (!ok) return;

        // Try server delete first; fallback to local removal on failure
        try {
          await requestV1(`/maps/${map.id}`, { method: "DELETE" });
          setMapsList((prev) => prev.filter((m) => m.id !== map.id));
          if (selectedMap && selectedMap.id === map.id) setSelectedMap(null);
          toast.success("Map deleted");
        } catch (err) {
          console.warn("Delete API failed, falling back to local remove", err);
          // fallback local behavior
          setMapsList((prev) => prev.filter((m) => m.id !== map.id));
          if (selectedMap && selectedMap.id === map.id) setSelectedMap(null);
          toast.success("Map removed (local)");
        }
        return;
      }
    } catch (err) {
      console.error("Map action error", err);
      toast.error(err.message || "Map action failed");
    }
  };

  // Create a new map immediately (optimistic local create with server persist attempt)
  const createNewMapImmediate = async () => {
    const newMap = {
      id: `local_${Date.now()}`,
      name: `New Map ${mapsList.length + 1}`,
      createdBy: "Operator",
      image: "",
      status: "Inactive",
      category: "",
      createdAt: new Date().toISOString().slice(0, 10),
    };
    // optimistic UI update
    setMapsList((prev) => [newMap, ...prev]);
    setSelectedMap(newMap);
    toast.success("Map added");

    // try persist to server, replace local id with server item on success
    try {
      const res = await requestV1("/maps", {
        method: "POST",
        body: JSON.stringify({
          name: newMap.name,
          createdBy: newMap.createdBy,
          image: newMap.image,
          status: newMap.status,
          category: newMap.category,
          createdAt: newMap.createdAt,
        }),
      });
      const created = res.item || { ...newMap, id: res.id || newMap.id };
      setMapsList((prev) => prev.map((m) => (m.id === newMap.id ? created : m)));
      setSelectedMap(created);
      toast.success("Map persisted");
    } catch (err) {
      console.warn("Persist new map failed, kept local map", err);
      toast.error("Map added locally (server persist failed)");
    }
  };

  // Activate a map (radio). Optimistic UI update, try to persist via API.
  const handleActivateMap = async (map) => {
    if (!map) return;
    try {
      // Optimistically update UI: mark chosen map Active, clear previous active status
      setMapsList((prev) =>
        prev.map((m) => {
          if (m.id === map.id) return { ...m, status: "Active" };
          // clear status for others that were Active
          if (m.status === "Active" && m.id !== map.id) return { ...m, status: "" };
          return m;
        }),
      );
      setSelectedMap(map);
      toast.success(`Activated map: ${map.name}`);

      // Persist change to server (best-effort)
      try {
        await requestV1(`/maps/${map.id}`, {
          method: "PATCH",
          body: JSON.stringify({ status: "Active" }),
        });
      } catch (err) {
        console.warn("Failed to persist map activation:", err);
        toast.error("Activation persisted locally (server sync failed)");
      }
    } catch (err) {
      console.error("Activation error", err);
      toast.error("Failed to activate map");
    }
  };
 
  const saveMapFromForm = async () => {
    // basic validation
    if (!mapForm.name.trim()) {
      toast.error("Map name required");
      return;
    }
    const payload = {
      name: mapForm.name.trim(),
      createdBy: mapForm.createdBy,
      image: mapForm.image,
      status: mapForm.status,
      category: mapForm.category,
      createdAt: mapForm.createdAt,
    };

    try {
      if (mapModalMode === "create") {
        // POST /maps
        try {
          const res = await requestV1("/maps", {
            method: "POST",
            body: JSON.stringify(payload),
          });
          const created = res.item || { ...payload, id: res.id || `map_${Date.now()}` };
          setMapsList((prev) => [created, ...prev]);
          setSelectedMap(created);
          toast.success("Map created");
        } catch (err) {
          // fallback local creation
          const created = { ...payload, id: `local_${Date.now()}` };
          setMapsList((prev) => [created, ...prev]);
          setSelectedMap(created);
          toast.success("Map created (local)");
        }
      } else if (mapModalMode === "edit") {
        const id = mapForm.id;
        try {
          const res = await requestV1(`/maps/${id}`, {
            method: "PATCH",
            body: JSON.stringify(payload),
          });
          const updated = res.item || { ...payload, id };
          setMapsList((prev) => prev.map((m) => (m.id === id ? { ...m, ...updated } : m)));
          if (selectedMap && selectedMap.id === id) setSelectedMap((s) => ({ ...s, ...updated }));
          toast.success("Map updated");
        } catch (err) {
          console.warn("Edit API failed, applying local update", err);
          setMapsList((prev) => prev.map((m) => (m.id === id ? { ...m, ...payload } : m)));
          if (selectedMap && selectedMap.id === id) setSelectedMap((s) => ({ ...s, ...payload }));
          toast.success("Map updated (local)");
        }
      }
      closeMapModal();
    } catch (err) {
      console.error("Save map error", err);
      toast.error(err.message || "Failed to save map");
    }
  };

  // active action radio state for the map action buttons
  const [activeMapAction, setActiveMapAction] = useState(null);

  // handler for testing: force error on next request
  const [forceError, setForceError] = useState(false);
  const handleTestError = () => setForceError((e) => !e);

  // helper to map battery level to CSS class
  const batteryClass = (() => {
    if (batteryLevel < 20) return "battery-red";
    if (batteryLevel < 50) return "battery-yellow";
    if (batteryLevel > 70) return "battery-green";
    return "battery-orange";
  })();

  // explicit color value for the icon (applied directly to the SVG via color prop)
  const batteryColor = (() => {
    if (batteryLevel < 20) return "#ff4d4d"; // red
    if (batteryLevel < 50) return "#f59e0b"; // yellow
    if (batteryLevel > 70) return "#10b981"; // green
    return "#f97316"; // mid (orange)
  })();

  const requestV1 = useCallback(
    async (path, options = {}) => {
      const headers = {
        Accept: "application/json",
        ...(options.headers || {}),
      };
      if (options.body && !headers["Content-Type"]) {
        headers["Content-Type"] = "application/json";
      }

      try {
        const response = await fetchWithAuth(`${API_V1_BASE}${path}`, {
          ...options,
          headers,
        });
        const data = await response.json().catch(() => ({}));
        if (response.status === 401 || response.status === 403) {
          clearAuthTokens();
          navigate("/");
          throw new Error(data.message || "Unauthorized");
        }
        if (!response.ok || data.success === false) {
          throw new Error(data.message || "Request failed");
        }
        return data;
      } catch (error) {
        // Bubble an explicit marker so mission flows can decide to retry/notify
        error.isApiError = true;
        if (error.message === "No access token available") {
          clearAuthTokens();
          navigate("/");
        }
        throw error;
      }
    },
    [navigate],
  );

  // Convenience wrapper for mission actions to ensure we capture errors in UI
  const missionRequest = useCallback(
    async (path, options = {}) => {
      return requestV1(path, options);
    },
    [requestV1],
  );

  const handleInitiateMission = useCallback(
    async (missionId) => {
      if (!missionId) {
        toast.error("Select a mission first");
        return;
      }
      setMissionActionLoading(true);
      try {
        const response = await missionRequest(`/missions/${missionId}/initiate`, {
          method: "POST",
        });
        const updated = response.item;
        if (updated) {
          setMissions((prev) =>
            prev.map((m) => (m.id === missionId ? { ...m, ...updated } : m)),
          );
          toast.success("Mission dispatched to robot");
        } else {
          toast.success("Mission dispatched");
        }
      } catch (error) {
        console.error("Mission initiate error", error);
        toast.error(error.message || "Failed to dispatch mission");
      } finally {
        setMissionActionLoading(false);
      }
    },
    [missionRequest],
  );

  const loadUsers = useCallback(async () => {
    setUsersLoading(true);
    setUsersError("");
    try {
      const response = await requestV1("/users");
      setUsers(response.items || []);
    } catch (error) {
      console.error("Users fetch error", error);
      setUsersError(error.message || "Failed to load users");
      toast.error(error.message || "Failed to load users");
    } finally {
      setUsersLoading(false);
    }
  }, [requestV1]);

  useEffect(() => {
    loadUsers();
  }, [loadUsers]);

  const handleResetUserPassword = useCallback(async () => {
    if (!selectedUserId) {
      toast.error("Select a user first");
      return;
    }
    setUserActionLoading(true);
    try {
      const response = await requestV1(
        `/users/${selectedUserId}/reset-password`,
        { method: "POST" },
      );
      if (response.item) {
        setUsers((prev) =>
          prev.map((user) =>
            user.id === response.item.id ? { ...user, ...response.item } : user,
          ),
        );
      }
      toast.success(response.message || "Password reset token generated");
    } catch (error) {
      console.error("Reset password error", error);
      toast.error(error.message || "Failed to reset password");
    } finally {
      setUserActionLoading(false);
    }
  }, [requestV1, selectedUserId]);

  // Chat functions
  const formatTimestamp = (value) => {
    const date = typeof value === "string" ? new Date(value) : value;
    return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  };

  const handleSendMessage = async (presetText = "") => {
    const messageText = (presetText || chatInput).trim();
    if (!messageText) return;
    const tempId = Date.now();
    const userMessage = {
      id: tempId,
      text: messageText,
      sender: "human",
      timestamp: new Date().toISOString(),
      status: "Sending",
    };
    setChatMessages((prev) => [...prev, userMessage]);
    setChatInput("");

    setIsTyping(true);
    try {
      const data = await requestV1("/chat/messages", {
        method: "POST",
        body: JSON.stringify({ text: messageText }),
      });
      setChatMessages((prev) => {
        const updated = prev.map((msg) =>
          msg.id === tempId
            ? { ...msg, status: data.message?.status || "Delivered" }
            : msg,
        );
        const next = data.reply ? [...updated, data.reply] : updated;
        return next;
      });
      if (Array.isArray(data.actions) && data.actions.length) {
        const actionNames = data.actions.map((a) => a.action || a.description).join(", ");
        toast.success(`Agent planned: ${actionNames}`);
      }
    } catch (error) {
      console.error("Chat send error", error);
      setChatMessages((prev) =>
        prev.map((msg) =>
          msg.id === tempId ? { ...msg, status: "Failed" } : msg,
        ),
      );
      toast.error(error.message || "Unable to send message");
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleMicClick = () => {
    setIsRecording(!isRecording);
    // TODO: Implement voice recording functionality
  };

  const handleSuggestionClick = (prompt) => {
    handleSendMessage(prompt);
  };

  const persistRobotSetting = useCallback(
    (key, value, previousValue) => {
      requestV1("/settings/robot", {
        method: "PATCH",
        body: JSON.stringify({ [key]: value }),
      }).catch((error) => {
        console.error("Robot setting update failed", error);
        toast.error(error.message || "Failed to update setting");
        setRobotSettingsState((prev) => ({ ...prev, [key]: previousValue }));
      });
    },
    [requestV1],
  );

  const toggleRobotSetting = (key) => {
    setRobotSettingsState((prev) => {
      const nextValue = !prev[key];
      persistRobotSetting(key, nextValue, prev[key]);
      return {
        ...prev,
        [key]: nextValue,
      };
    });
  };

  const handleAccountChange = (field, value) => {
    setAccountProfile((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const loadAccountProfile = useCallback(async () => {
    try {
      const response = await fetchWithAuth(`${API_BASE}/api/user/profile`);
      const data = await response.json().catch(() => ({}));

      if (response.status === 401 || response.status === 403) {
        clearAuthTokens();
        navigate("/");
        return;
      }

      if (response.ok && data.success) {
        setAccountProfile({
          fullName: data.username || "",
          email: data.email || "",
          team: data.company || "",
          shift: data.amr_type || "",
        });
        setProfileError("");
      } else {
        setProfileError(data.message || "Failed to load profile.");
      }
    } catch (error) {
      console.error("Failed to load profile", error);
      if (error.message === "No access token available") {
        clearAuthTokens();
        navigate("/");
        return;
      }
      setProfileError("Unable to load profile.");
    }
  }, [navigate]);

  useEffect(() => {
    loadAccountProfile();
  }, [loadAccountProfile]);

  const handleSaveProfile = async () => {
    setProfileSaving(true);
    setProfileError("");
    setProfileSuccess("");

    try {
      const response = await fetchWithAuth(`${API_BASE}/api/user/profile`, {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          email: accountProfile.email,
          company: accountProfile.team,
          amrType: accountProfile.shift,
        }),
      });

      const data = await response.json().catch(() => ({}));

      if (response.status === 401 || response.status === 403) {
        clearAuthTokens();
        navigate("/");
        return;
      }

      if (!response.ok || !data.success) {
        throw new Error(data.message || "Failed to update profile.");
      }

      toast.success("Profile updated");
      setProfileSuccess("Profile saved successfully.");
    } catch (error) {
      console.error("Failed to save profile", error);
      if (error.message === "No access token available") {
        clearAuthTokens();
        navigate("/");
        return;
      }
      setProfileError(error.message || "Unable to save profile.");
    } finally {
      setProfileSaving(false);
      setTimeout(() => setProfileSuccess(""), 3000);
    }
  };

  const persistSecurityPref = useCallback(
    (key, value, previousValue) => {
      requestV1("/settings/security", {
        method: "PATCH",
        body: JSON.stringify({ [key]: value }),
      }).catch((error) => {
        console.error("Security pref update failed", error);
        toast.error(error.message || "Failed to update security preference");
        setSecurityPreferences((prev) => ({ ...prev, [key]: previousValue }));
      });
    },
    [requestV1],
  );

  const toggleSecurityPref = (key) => {
    setSecurityPreferences((prev) => {
      const nextValue = !prev[key];
      persistSecurityPref(key, nextValue, prev[key]);
      return {
        ...prev,
        [key]: nextValue,
      };
    });
  };

  const persistIntegrationStatus = useCallback(
    (id, status, previousStatus) => {
      requestV1(`/settings/integrations/${id}`, {
        method: "PATCH",
        body: JSON.stringify({ status }),
      }).catch((error) => {
        console.error("Integration status update failed", error);
        toast.error(error.message || "Failed to update integration");
        setIntegrationItems((items) =>
          items.map((item) =>
            item.id === id ? { ...item, status: previousStatus } : item,
          ),
        );
      });
    },
    [requestV1],
  );

  const toggleIntegrationStatus = (id) => {
    setIntegrationItems((items) =>
      items.map((item) => {
        if (item.id !== id) {
          return item;
        }
        const nextStatus =
          item.status === "Connected" ? "Disconnected" : "Connected";
        persistIntegrationStatus(id, nextStatus, item.status);
        return {
          ...item,
          status: nextStatus,
        };
      }),
    );
  };

  const lineChartSize = { width: 420, height: 180 };
  const buildLinePath = (series, key) => {
    if (!series.length) return "";
    const max = Math.max(...series.map((point) => point[key]));
    const min = Math.min(...series.map((point) => point[key]));
    const range = max - min || 1;
    const stepX =
      series.length > 1
        ? lineChartSize.width / (series.length - 1)
        : lineChartSize.width;
    return series
      .map((point, index) => {
        const x = index * stepX;
        const normalized = (point[key] - min) / range;
        const y = lineChartSize.height - normalized * lineChartSize.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  const buildSimplePath = (points, size) => {
    if (!points.length) return "";
    const max = Math.max(...points);
    const min = Math.min(...points);
    const range = max - min || 1;
    const step =
      points.length > 1 ? size.width / (points.length - 1) : size.width;
    return points
      .map((value, index) => {
        const x = index * step;
        const normalized = (value - min) / range;
        const y = size.height - normalized * size.height;
        return `${x},${y}`;
      })
      .join(" ");
  };

  useEffect(() => {
    let cancelled = false;

    const loadStats = async () => {
      try {
        if (!cancelled) {
          setStatsLoading(true);
          setStatsError("");
        }

        const payload = await requestV1("/stats");
        if (!cancelled) {
          const statsPayload = payload.data || FALLBACK_STATS;
          setStatsData({
            overview: statsPayload.overview || FALLBACK_STATS.overview,
            missionTrend:
              statsPayload.missionTrend || FALLBACK_STATS.missionTrend,
            monthlyMovement:
              statsPayload.monthlyMovement || FALLBACK_STATS.monthlyMovement,
            batterySeries:
              statsPayload.batterySeries || FALLBACK_STATS.batterySeries,
            batteryStatus:
              statsPayload.batteryStatus || FALLBACK_STATS.batteryStatus,
            turns: statsPayload.turns || FALLBACK_STATS.turns,
          });
        }
      } catch (err) {
        console.error("Stats fetch error", err);
        if (!cancelled) {
          setStatsData(FALLBACK_STATS);
          setStatsError("Using cached telemetry");
        }
      } finally {
        if (!cancelled) {
          setStatsLoading(false);
        }
      }
    };

    loadStats();
    const intervalId = setInterval(loadStats, 60000);
    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, [requestV1]);

  useEffect(() => {
    let cancelled = false;

    const loadCreateData = async () => {
      try {
        const [mapsRes, zonesRes, waypointsRes, missionsRes, usersRes] =
          await Promise.all([
            requestV1("/maps"),
            requestV1("/zones"),
            requestV1("/waypoints"),
            requestV1("/missions"),
            requestV1("/users"),
          ]);

        if (cancelled) {
          return;
        }

        if (Array.isArray(mapsRes.items) && mapsRes.items.length) {
          setMapsList(mapsRes.items);
          setSelectedMap((prev) => {
            if (prev) {
              const stillExists = mapsRes.items.find((m) => m.id === prev.id);
              if (stillExists) {
                return stillExists;
              }
            }
            return mapsRes.items[0] || null;
          });
        }
        if (Array.isArray(zonesRes.items)) {
          setZones(zonesRes.items);
        }
        if (Array.isArray(waypointsRes.items)) {
          setWaypoints(waypointsRes.items);
        }
        if (Array.isArray(missionsRes.items)) {
          setMissions(missionsRes.items);
        }
        if (Array.isArray(usersRes.items)) {
          setUsers(usersRes.items);
        }
      } catch (error) {
        console.error("Failed to load workspace data", error);
      }
    };

    loadCreateData();
    return () => {
      cancelled = true;
    };
  }, [requestV1]);

  useEffect(() => {
    let cancelled = false;

    const loadMonitorData = async () => {
      try {
        const [analyticsRes, diagnosticsRes, logsRes, missionLogsRes, bagsRes] =
          await Promise.all([
            requestV1("/monitor/analytics"),
            requestV1("/monitor/diagnostics"),
            requestV1("/monitor/logs"),
            requestV1("/monitor/mission-logs"),
            requestV1("/monitor/robot-bags"),
          ]);

        if (cancelled) {
          return;
        }

        const analyticsPayload = analyticsRes.data || {};
        if (Array.isArray(analyticsPayload.summary)) {
          setAnalyticsSummary(analyticsPayload.summary);
        }
        if (Array.isArray(analyticsPayload.series)) {
          setAnalyticsSeries(analyticsPayload.series);
        }
        if (Array.isArray(analyticsPayload.alerts)) {
          setAnalyticsAlerts(analyticsPayload.alerts);
        }
        if (Array.isArray(diagnosticsRes.items)) {
          setDiagnosticsPanels(diagnosticsRes.items);
        }
        if (Array.isArray(logsRes.items)) {
          setLogEvents(logsRes.items);
        }
        if (Array.isArray(missionLogsRes.items)) {
          setMissionHistory(missionLogsRes.items);
        }
        if (Array.isArray(bagsRes.items)) {
          setBagFiles(bagsRes.items);
        }
      } catch (error) {
        console.error("Failed to load monitor data", error);
      }
    };

    loadMonitorData();
    const intervalId = setInterval(loadMonitorData, 60000);
    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, [requestV1]);

  useEffect(() => {
    let cancelled = false;

    const loadSettingsAndChat = async () => {
      try {
        const [
          robotRes,
          securityRes,
          securityEventsRes,
          integrationsRes,
          appearanceRes,
          chatRes,
        ] = await Promise.all([
          requestV1("/settings/robot"),
          requestV1("/settings/security"),
          requestV1("/settings/security/events"),
          requestV1("/settings/integrations"),
          requestV1("/settings/appearance"),
          requestV1("/chat/history"),
        ]);

        if (cancelled) {
          return;
        }

        if (robotRes.data) {
          setRobotSettingsState((prev) => ({ ...prev, ...robotRes.data }));
        }
        if (securityRes.data) {
          setSecurityPreferences((prev) => ({ ...prev, ...securityRes.data }));
        }
        if (Array.isArray(securityEventsRes.items)) {
          setSecurityEvents(securityEventsRes.items);
        }
        if (Array.isArray(integrationsRes.items)) {
          setIntegrationItems(integrationsRes.items);
        }
        if (appearanceRes.data?.theme) {
          setSelectedTheme(appearanceRes.data.theme);
        }
        if (Array.isArray(chatRes.items) && chatRes.items.length) {
          setChatMessages(chatRes.items);
        }
      } catch (error) {
        console.error("Failed to load settings/chat data", error);
      }
    };

    loadSettingsAndChat();
    return () => {
      cancelled = true;
    };
  }, [requestV1]);

  const {
    overview,
    missionTrend,
    monthlyMovement,
    batterySeries,
    batteryStatus,
    turns,
  } = statsData;
  const analyticsChartSize = { width: 280, height: 80 };
  const analyticsPath = buildSimplePath(analyticsSeries, analyticsChartSize);
  const overviewDeltaLabel =
    typeof overview.deltaKm === "number"
      ? overview.deltaKm.toFixed(1)
      : overview.deltaKm;
  const batteryVoltagePath = buildLinePath(batterySeries, "voltage");
  const batteryPowerPath = buildLinePath(batterySeries, "power");
  const totalTurns = (turns.left || 0) + (turns.right || 0) || 1;
  const leftTurnPercent = Math.round(((turns.left || 0) / totalTurns) * 100);
  const rightTurnPercent = Math.round(((turns.right || 0) / totalTurns) * 100);
  const monthlyMaxKm = monthlyMovement.length
    ? Math.max(...monthlyMovement.map((m) => m.km))
    : 1;
  const totalMonthlyKm = monthlyMovement.reduce(
    (sum, entry) => sum + (entry.km || 0),
    0,
  );
  const avgMonthlyKm = monthlyMovement.length
    ? (totalMonthlyKm / monthlyMovement.length).toFixed(1)
    : "0.0";

  // breadcrumb parts for header (Home → Page → Item)
  const latestMessage = chatMessages[chatMessages.length - 1];
  const breadcrumbParts = ["Home"];
  if (rightPage)
    breadcrumbParts.push(
      rightPage.charAt(0).toUpperCase() + rightPage.slice(1),
    );
  if (rightPage === "maps" && selectedMap)
    breadcrumbParts.push(selectedMap.name);

  // stable id to avoid duplicate toasts for emergency toggle
  const EMERGENCY_TOAST_ID = "emergency-toast";

  // centralized handler to toggle emergency and show single toast
  const handleEmergencyToggle = React.useCallback(() => {
    setEmergencyClicked((prev) => {
      const next = !prev;
      // ensure any previous emergency toast is removed first
      toast.dismiss(EMERGENCY_TOAST_ID);
      if (next) {
        toast.error("Emergency stop engaged", { id: EMERGENCY_TOAST_ID });
      } else {
        toast.success("Emergency stop released", { id: EMERGENCY_TOAST_ID });
      }
      return next;
    });
  }, []);

  // Runtime diagnostics: show tiny startup toast and report unhandled errors
  useEffect(() => {
    // small startup hint so we know React mounted
    toast.dismiss("app-start");
    toast.success("App initialized", { id: "app-start", duration: 1500 });

    const onUnhandledRejection = (ev) => {
      console.error("Unhandled promise rejection:", ev.reason);
      try {
        toast.error(`Unhandled rejection: ${String(ev.reason).slice(0, 80)}`, { duration: 4000 });
      } catch {}
    };
    const onError = (message, source, lineno, colno, error) => {
      console.error("Window error:", message, { source, lineno, colno, error });
      try {
        toast.error(`Error: ${String(message).slice(0, 80)}`, { duration: 4000 });
      } catch {}
      // return false to allow default browser handling
      return false;
    };

    window.addEventListener("unhandledrejection", onUnhandledRejection);
    window.addEventListener("error", onError);
    return () => {
      window.removeEventListener("unhandledrejection", onUnhandledRejection);
      window.removeEventListener("error", onError);
    };
  }, []);

  // allow unlocking via Escape to avoid getting locked out
  useEffect(() => {
    const onKey = (e) => {
      if (!isLocked) return;
      if (e.key === "Escape") {
        handleToggleLock();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [isLocked, handleToggleLock]);

  return (
    <div
      ref={layoutRef}
      className={`main-container ${rightPage ? "has-right-pane" : ""} ${
        isLocked ? "is-locked" : ""
      }`}
    >
      <header className="mp-header">
        {/* left header group (kept minimal) */}
        <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
          <div
            className={`bridge-chip ${
              bridgeStatus.connected ? "ok" : "warn"
            }`}
            title={
              bridgeStatus.error
                ? `Bridge: ${bridgeStatus.error}`
                : bridgeStatus.connected
                ? `Connected to AMR bridge (${bridgeStatus.endpoint})`
                : `Bridge offline (${bridgeStatus.endpoint})`
            }
          >
            <span className="bridge-dot" />
            <div className="bridge-chip-text">
              <div className="bridge-chip-label">AMR Bridge</div>
              <div className="bridge-chip-sub">
                {bridgeStatus.connected ? "Connected" : "Offline"}
                {bridgeStatus.endpoint ? ` · ${bridgeStatus.endpoint}` : ""}
              </div>
            </div>
            <button
              type="button"
              className="bridge-retry-btn"
              onClick={() => {
                toast.dismiss("bridge-retry");
                toast.success("Retrying bridge connection…", { id: "bridge-retry", duration: 1500 });
                startBridgeConnection();
              }}
            >
              Retry
            </button>
          </div>
        </div>

        {/* Header-centered Emergency Stop */}
        <div className="header-center-emergency">
          <button
            onClick={handleEmergencyToggle}
            aria-pressed={emergencyClicked}
            className={
              emergencyClicked
                ? "header-emergency-btn clicked"
                : "header-emergency-btn"
            }
          >
            <span className="header-emergency-dot" aria-hidden="true">
              <span className="header-emergency-dot-inner" />
            </span>
            <span className="header-emergency-label">Emergency Stop</span>
          </button>
        </div>

        {/* Dropdown with play/pause */}
        <div className="dropdown-wrap">
          <div
            className="dropdown-btn"
            role="button"
            tabIndex={0}
            onClick={() => setIsDropdownOpen((s) => !s)}
            aria-expanded={isDropdownOpen}
          >
            <button
              onClick={(e) => {
                e.stopPropagation();
                setIsPlaying((p) => !p);
              }}
              title={isPlaying ? "Pause" : "Play"}
              className={isPlaying ? "play-btn playing" : "play-btn"}
              aria-pressed={isPlaying}
            >
              {isPlaying ? <FaPause /> : <FaPlay />}
            </button>

            <span>Robot: UI EMERGENCY</span>
            <span className="caret">▾</span>
          </div>

          {isDropdownOpen && (
            <div role="dialog" className="dropdown-menu">
              <div className="dropdown-title">Robot: UI EMERGENCY</div>
              <div className="dropdown-body">
                There is no mission running. Initiate a mission to get started.
              </div>
            </div>
          )}
        </div>

        {/* header right: battery + fullscreen */}
        <div className="header-right">
          <button
            className={`battery ${batteryClass}`}
            type="button"
            onClick={() => setBatteryModalOpen(true)}
            title="Battery status"
          >
            <FaBatteryHalf className="battery-icon" color={batteryColor} />
            <div className="battery-percent">{batteryLevel}%</div>
          </button>
          <button
            onClick={toggleFullScreen}
            title={isFullScreen ? "Exit Fullscreen" : "Fullscreen"}
            className="fullscreen-btn"
            type="button"
          >
            {isFullScreen ? <FaCompress /> : <FaExpand />}
          </button>

          {/* Lock icon: use top-level isLocked and handleToggleLock defined earlier */}
          <button
            type="button"
            className="lock-toggle-btn"
            aria-pressed={isLocked}
            aria-label={isLocked ? "Unlock console" : "Lock console"}
            onMouseDown={(e) => e.preventDefault()} /* prevent focusing on mouse down */
            onClick={handleToggleLock}
            title={isLocked ? "Unlock console" : "Lock console"}
          >
            {isLocked ? <FaLock /> : <FaUnlock />}
          </button>

          {/* Logout button — icon-only for compact header */}
          <button
            type="button"
            className="logout-btn"
            onClick={handleLogout}
            title="Log out"
            aria-label="Log out"
          >
            <FaSignOutAlt />
          </button>
        </div>
      </header>

      <div className="content-wrap">
        <Sidebar
          onSelect={handleSidebarSelect}
          onBack={() => {
            // close the right-side page if open; otherwise no-op for now
            if (rightPage) setRightPage(null);
          }}
        />
        {/* Right pane: shows the selected page in the right half of the screen */}
        {rightPage && (
          <aside className="right-pane" role="region" aria-label="Right pane">
            <div className="right-pane-header">
              <strong>
                {rightPage.charAt(0).toUpperCase() + rightPage.slice(1)}
              </strong>
              <button
                className="right-pane-close"
                onClick={() => setRightPage(null)}
                aria-label="Close"
              >
                ✕
              </button>
            </div>

            <div className="right-pane-body">
              {/* Maps page: unified search + create + table */}
              {rightPage === "maps" && (
                <div style={{ display: "flex", flexDirection: "column", gap: 8 }}>
                  <div style={{ display: "flex", gap: 8, flexWrap: "wrap", alignItems: "center" }}>
                    <select
                      aria-label="Search field"
                      value={mapSearchField}
                      onChange={(e) => setMapSearchField(e.target.value)}
                      style={{ padding: "6px 8px" }}
                    >
                      <option value="any">Search By</option>
                      <option value="name">Name</option>
                      <option value="createdBy">Created By</option>
                      <option value="category">Category</option>
                      <option value="createdAt">Created At</option>
                      <option value="status">Status</option>
                    </select>

                    <input
                      value={mapSearchTerm}
                      onChange={(e) => setMapSearchTerm(e.target.value)}
                      placeholder="Type to search..."
                      style={{ padding: "6px 8px", minWidth: 220 }}
                    />

                    <div>
                      <button
                        onClick={createNewMapImmediate}
                        aria-label="Create new map"
                        title="+ Create New Map"
                        style={{
                          background: "#0b74d1",
                          color: "#fff",
                          padding: "8px 12px",
                          borderRadius: 8,
                          border: "none",
                          cursor: "pointer",
                          boxShadow: "0 6px 18px rgba(11,116,209,0.16)",
                          display: "inline-flex",
                          alignItems: "center",
                          gap: 8,
                          fontWeight: 700,
                        }}
                      >
                        <FaPlus />
                        <span> Create New Map</span>
                      </button>
                    </div>
                  </div>
                        
                  <div style={{ borderTop: "1px solid #e6eef2", marginTop: 8 }} />

                  <div style={{ overflowY: "auto", maxHeight: 420 }}>
                    <table style={{ width: "100%", borderCollapse: "collapse", marginTop: 8 }}>
                      <thead style={{ background: "#f8fafc" }}>
                        <tr>
                          <th style={{ textAlign: "center", padding: 12, width: 72 }}>Active</th>
                          <th style={{ textAlign: "left", padding: 12 }}>Name</th>
                          <th style={{ textAlign: "left", padding: 12 }}>Created By</th>
                          <th style={{ textAlign: "left", padding: 12 }}>Created At</th>
                          <th style={{ textAlign: "right", padding: 12 }}>Status</th>
                          <th style={{ textAlign: "right", padding: 12, width: 160 }}>Actions</th>
                        </tr>
                      </thead>
                      <tbody>
                        {(() => {
                          const term = (mapSearchTerm || "").trim().toLowerCase();
                          const field = mapSearchField;
                          const filtered = mapsList.filter((m) => {
                            if (!term) return true;
                            if (field === "any") {
                              const hay = `${m.name||""} ${m.createdBy||""} ${m.category||""} ${m.createdAt||""} ${m.status||""}`.toLowerCase();
                              return hay.includes(term);
                            }
                            return String(m[field] || "").toLowerCase().includes(term);
                          });
                          return filtered.map((m) => (
                            <tr key={m.id} onClick={() => setSelectedMap(m)} style={{ cursor: "pointer", background: selectedMap?.id === m.id ? "rgba(3,48,80,0.04)" : "transparent" }}>
                              <td style={{ padding: 12, borderBottom: "1px solid #eef2f6", textAlign: "center" }}>
                                <input
                                  type="radio"
                                  name="activeMap"
                                  checked={selectedMap?.id === m.id}
                                  onChange={(e) => {
                                    e.stopPropagation();
                                    handleActivateMap(m);
                                  }}
                                  aria-label={`Activate ${m.name}`}
                                />
                              </td>
                              <td style={{ padding: 12, borderBottom: "1px solid #eef2f6", fontWeight: 700 }}>{m.name}</td>
                              <td style={{ padding: 12, borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{m.createdBy}</td>
                              <td style={{ padding: 12, borderBottom: "1px solid #eef2f6", color: "#6b7280" }}>{m.createdAt || "—"}</td>
                              <td style={{ padding: 12, borderBottom: "1px solid #eef2f6", textAlign: "right" }}>
                                <span
                                  style={{
                                    background:
                                      (String(m.status || "").toLowerCase() === "active")
                                        ? "#10b981" // green for Active
                                        : "#ef4444ff", // red for Inactive / other
                                    color: "#fff",
                                    padding: "2px 8px",
                                    borderRadius: 8,
                                    textTransform: "capitalize",
                                  }}
                                >
                                  {m.status ? m.status : "Inactive"}
                                </span>
                              </td>
                              <td style={{ padding: 12, borderBottom: "1px solid #eef2f6", display: "flex", gap: 8, justifyContent: "flex-end" }} onClick={(e) => e.stopPropagation()}>
                                <button title="Edit" onClick={() => handleMapAction("edit", m)} className="ghost-btn"><FaEdit /></button>
                                <button title="Delete" onClick={() => handleMapAction("delete", m)} className="ghost-btn"><FaTrash /></button>
                              </td>
                            </tr>
                          ));
                        })()}
                      </tbody>
                    </table>
                  </div>
                </div>
              )}

              {/* Zones page: search, create button, and table matching screenshot layout */}
              {rightPage === "zones" && (
                <div
                  style={{ display: "flex", flexDirection: "column", gap: 12 }}
                >
                  <div
                    style={{
                      display: "flex",
                      justifyContent: "space-between",
                      alignItems: "center",
                      gap: 12,
                    }}
                  >
                    <div
                      style={{ display: "flex", alignItems: "center", gap: 8 }}
                    >
                      <label
                        style={{
                          fontSize: 13,
                          color: "#475569",
                          fontWeight: 600,
                        }}
                      >
                        Search Zone By
                      </label>
                      <select style={{ padding: "6px 8px" }}>
                       <option value="Search By">Search By</option>
                       <option value="name">Name</option>
                       <option value="createdBy">Created By</option>
                       <option value="category">Category</option>
                       <option value="createdAt">Created At</option>
                       <option value="status">Status</option>
                      </select>
                      <input
                        placeholder="Search zone..."
                        style={{
                          padding: "8px 10px",
                          borderRadius: 8,
                          border: "1px solid #e6eef2",
                          minWidth: 220,
                        }}
                      />
                    </div>

                    <div>
                      <button
                        onClick={() => setZoneFormOpen((v) => !v)}
                        style={{
                          background: "#0b74d1",
                          color: "#fff",
                          padding: "10px 14px",
                          borderRadius: 8,
                          border: "none",
                          cursor: "pointer",
                          boxShadow: "0 6px 18px rgba(11,116,209,0.16)",
                        }}
                      >
                        + Create New Zone
                      </button>
                    </div>
                  </div>

                  {zoneFormOpen && (
                    <div className="zone-form">
                      <div className="zone-form-row">
                        <label>
                          Name
                          <input
                            value={zoneForm.name}
                            onChange={(e) =>
                              setZoneForm((prev) => ({
                                ...prev,
                                name: e.target.value,
                              }))
                            }
                            placeholder="Zone name"
                          />
                        </label>
                        <label>
                          Category
                          <select
                            value={zoneForm.category}
                            onChange={(e) =>
                              setZoneForm((prev) => ({
                                ...prev,
                                category: e.target.value,
                              }))
                            }
                          >
                            <option value="Safe">Safe</option>
                            <option value="Caution">Caution</option>
                            <option value="No-Go">No-Go</option>
                          </select>
                        </label>
                        <label>
                          Geometry
                          <input
                            value={zoneForm.geometry}
                            onChange={(e) =>
                              setZoneForm((prev) => ({
                                ...prev,
                                geometry: e.target.value,
                              }))
                            }
                            placeholder="Polygon(...)"
                          />
                        </label>
                      </div>
                      <div className="zone-form-footer">
                        <label className="toggle-row">
                          <input
                            type="checkbox"
                            checked={zoneForm.active}
                            onChange={() =>
                              setZoneForm((prev) => ({
                                ...prev,
                                active: !prev.active,
                              }))
                            }
                          />
                          <div>
                            <strong>Zone Enabled</strong>
                            <p>
                              {zoneForm.active
                                ? "Robots can enter"
                                : "Robots must avoid"}
                            </p>
                          </div>
                        </label>
                        <div className="zone-form-actions">
                          <button
                            className="ghost-btn"
                            type="button"
                            onClick={() => {
                              setZoneFormOpen(false);
                              setZoneForm({
                                name: "",
                                category: "Safe",
                                geometry: "",
                                active: true,
                              });
                            }}
                          >
                            Cancel
                          </button>
                          <button
                            className="primary-btn"
                            type="button"
                            onClick={async () => {
                              if (!zoneForm.name.trim()) {
                                toast.error("Enter a zone name");
                                return;
                              }
                              const payload = {
                                name: zoneForm.name.trim(),
                                category: zoneForm.category,
                                geometry: zoneForm.geometry || "Polygon(...)",
                                active: zoneForm.active,
                              };
                              try {
                                const response = await requestV1("/zones", {
                                  method: "POST",
                                  body: JSON.stringify(payload),
                                });
                                const createdZone = response.item || payload;
                                setZones((prev) => [createdZone, ...prev]);
                                setZoneForm({
                                  name: "",
                                  category: "Safe",
                                  geometry: "",
                                  active: true,
                                });
                                setZoneFormOpen(false);
                                toast.success("Zone created");
                              } catch (error) {
                                console.error("Zone create error", error);
                                toast.error(error.message || "Failed to create zone");
                              }
                            }}
                          >
                            Save Zone
                          </button>
                        </div>
                      </div>
                    </div>
                  )}

                  <div
                    style={{ borderTop: "1px solid #e6eef2", marginTop: 4 }}
                  />

                  {/* single-column layout: table occupies full available width (empty body) */}
                  <div>
                    <div
                      style={{
                        background: "#fff",
                        borderRadius: 8,
                        boxShadow: "0 1px 3px rgba(2,6,23,0.06)",
                        overflow: "hidden",
                      }}
                    >
                      <div
                        style={{
                          padding: "12px 16px",
                          borderBottom: "1px solid #eef2f6",
                          display: "flex",
                          alignItems: "center",
                          gap: 12,
                        }}
                      >
                        <div
                          style={{ flex: 1, fontWeight: 700, color: "#0f172a" }}
                        >
                          Zones
                        </div>
                        <div style={{ color: "#94a3b8", fontSize: 13 }}>
                          Rows per page: 5
                        </div>
                      </div>

                      <div style={{ padding: "8px 16px" }}>
                        <table
                          style={{ width: "100%", borderCollapse: "collapse" }}
                        >
                          <thead
                            style={{
                              background: "#fafafa",
                              color: "#475569",
                              fontSize: 13,
                            }}
                          >
                            <tr>
                              <th
                                style={{
                                  textAlign: "left",
                                  padding: "12px 8px",
                                }}
                              >
                                Name
                              </th>
                              <th
                                style={{
                                  textAlign: "left",
                                  padding: "12px 8px",
                                }}
                              >
                                Category
                              </th>
                              <th
                                style={{
                                  textAlign: "center",
                                  padding: "12px 8px",
                                }}
                              >
                                Active
                              </th>
                              <th
                                style={{
                                  textAlign: "left",
                                  padding: "12px 8px",
                                }}
                              >
                                Geometry
                              </th>
                              <th
                                style={{
                                  textAlign: "right",
                                  padding: "12px 8px",
                                }}
                              >
                                Created At
                              </th>
                            </tr>
                          </thead>
                          <tbody>
                            {zones.map((zone) => (
                              <tr key={zone.id}>
                                <td
                                  style={{
                                    padding: "12px 8px",
                                    borderBottom: "1px solid #eef2f6",
                                    fontWeight: 700,
                                  }}
                                >
                                  {zone.name}
                                </td>
                                <td
                                  style={{
                                    padding: "12px 8px",
                                    borderBottom: "1px solid #eef2f6",
                                  }}
                                >
                                  {zone.category}
                                </td>
                                <td
                                  style={{
                                    padding: "12px 8px",
                                    borderBottom: "1px solid #eef2f6",
                                    textAlign: "center",
                                  }}
                                >
                                  <span
                                    style={{
                                      background: zone.active
                                        ? "#d1fae5"
                                        : "#fee2e2",
                                      color: zone.active
                                        ? "#047857"
                                        : "#b91c1c",
                                      padding: "4px 8px",
                                      borderRadius: 999,
                                      fontSize: 12,
                                    }}
                                  >
                                    {zone.active ? "Active" : "Disabled"}
                                  </span>
                                </td>
                                <td
                                  style={{
                                    padding: "12px 8px",
                                    borderBottom: "1px solid #eef2f6",
                                    color: "#6b7280",
                                  }}
                                >
                                  {zone.geometry}
                                </td>
                                <td
                                  style={{
                                    padding: "12px 8px",
                                    borderBottom: "1px solid #eef2f6",
                                    textAlign: "right",
                                    color: "#6b7280",
                                  }}
                                >
                                  {zone.createdAt}
                                </td>
                              </tr>
                            ))}
                          </tbody>
                        </table>
                      </div>
                    </div>
                  </div>
                </div>
              )}

              {/* Waypoints page: show map/file path on left and waypoints table on right */}
              {rightPage === "waypoints" && (
                <div
                  style={{ display: "flex", flexDirection: "column", gap: 12 }}
                >
                  <div
                    style={{
                      display: "flex",
                      justifyContent: "space-between",
                      alignItems: "center",
                    }}
                  >
                    <div
                      style={{ display: "flex", alignItems: "center", gap: 8 }}
                    >
                      <label
                        style={{
                          fontSize: 13,
                          color: "#475569",
                          fontWeight: 600,
                        }}
                      >
                        Search Waypoint By
                      </label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Category</option>
                      </select>
                      <input
                        placeholder="Search waypoint..."
                        style={{
                          padding: "8px 10px",
                          borderRadius: 8,
                          border: "1px solid #e6eef2",
                          minWidth: 220,
                        }}
                      />
                    </div>
                    <div>
                      <button
                        style={{
                          background: "#0b74d1",
                          color: "#fff",
                          padding: "10px 14px",
                          borderRadius: 8,
                          border: "none",
                          cursor: "pointer",
                        }}
                        onClick={() => setWaypointFormOpen((v) => !v)}
                      >
                        + Create New Waypoint
                      </button>
                    </div>
                  </div>

                  {waypointFormOpen && (
                    <div className="waypoint-form">
                      <div className="zone-form-row">
                        <label>
                          Name
                          <input
                            value={waypointForm.name}
                            onChange={(e) =>
                              setWaypointForm((prev) => ({
                                ...prev,
                                name: e.target.value,
                              }))
                            }
                            placeholder="Waypoint name"
                          />
                        </label>
                        <label>
                          Category
                          <select
                            value={waypointForm.category}
                            onChange={(e) =>
                              setWaypointForm((prev) => ({
                                ...prev,
                                category: e.target.value,
                              }))
                            }
                          >
                            <option value="Nav">Nav</option>
                            <option value="Inspect">Inspect</option>
                            <option value="Charge">Charge</option>
                          </select>
                        </label>
                        <label>
                          Geometry
                          <input
                            value={waypointForm.geom}
                            onChange={(e) =>
                              setWaypointForm((prev) => ({
                                ...prev,
                                geom: e.target.value,
                              }))
                            }
                            placeholder="Point(x y)"
                          />
                        </label>
                      </div>
                      <label>
                        Notes
                        <input
                          value={waypointForm.notes}
                          onChange={(e) =>
                            setWaypointForm((prev) => ({
                              ...prev,
                              notes: e.target.value,
                            }))
                          }
                          placeholder="Optional operator note"
                        />
                      </label>
                      <div className="zone-form-footer">
                        <label className="toggle-row">
                          <input
                            type="checkbox"
                            checked={waypointForm.active}
                            onChange={() =>
                              setWaypointForm((prev) => ({
                                ...prev,
                                active: !prev.active,
                              }))
                            }
                          />
                          <div>
                            <strong>Waypoint Active</strong>
                            <p>
                              {waypointForm.active
                                ? "Included in missions"
                                : "Hidden from routing"}
                            </p>
                          </div>
                        </label>
                        <div className="zone-form-actions">
                          <button
                            className="ghost-btn"
                            type="button"
                            onClick={() => {
                              setWaypointFormOpen(false);
                              setWaypointForm({
                                name: "",
                                category: "Nav",
                                geom: "",
                                notes: "",
                                active: true,
                              });
                            }}
                          >
                            Cancel
                          </button>
                          <button
                            className="primary-btn"
                            type="button"
                            onClick={async () => {
                              if (!waypointForm.name.trim()) {
                                toast.error("Enter a waypoint name");
                                return;
                              }
                              if (!waypointForm.geom.trim()) {
                                toast.error("Add waypoint coordinates");
                                return;
                              }
                              const payload = {
                                name: waypointForm.name.trim(),
                                category: waypointForm.category,
                                geom: waypointForm.geom || "Point(0 0)",
                                notes: waypointForm.notes,
                                active: waypointForm.active,
                              };
                              try {
                                const response = await requestV1("/waypoints", {
                                  method: "POST",
                                  body: JSON.stringify(payload),
                                });
                                const createdWp = response.item || payload;
                                setWaypoints((prev) => [createdWp, ...prev]);
                                setSelectedWaypointId(createdWp.id);
                                setWaypointForm({
                                  name: "",
                                  category: "Nav",
                                  geom: "",
                                  notes: "",
                                  active: true,
                                });
                                setWaypointFormOpen(false);
                                toast.success("Waypoint created");
                              } catch (error) {
                                console.error("Waypoint create error", error);
                                toast.error(
                                  error.message || "Failed to create waypoint",
                                );
                              }
                            }}
                          >
                            Save Waypoint
                          </button>
                        </div>
                      </div>
                    </div>
                  )}

                  <div
                    style={{
                      background: "#fff",
                      borderRadius: 8,
                      boxShadow: "0 1px 3px rgba(2,6,23,0.06)",
                      overflow: "hidden",
                    }}
                  >
                    <div
                      style={{
                        padding: "12px 16px",
                        borderBottom: "1px solid #eef2f6",
                        display: "flex",
                        alignItems: "center",
                        gap: 12,
                      }}
                    >
                      <div
                        style={{ flex: 1, fontWeight: 700, color: "#0f172a" }}
                      >
                        Waypoints
                      </div>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>
                        Rows per page: 10
                      </div>
                    </div>
                    <div style={{ padding: "8px 16px" }}>
                      <table
                        style={{ width: "100%", borderCollapse: "collapse" }}
                      >
                        <thead
                          style={{
                            background: "#fafafa",
                            color: "#475569",
                            fontSize: 13,
                          }}
                        >
                          <tr>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Name
                            </th>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Category
                            </th>
                            <th
                              style={{
                                textAlign: "center",
                                padding: "12px 8px",
                              }}
                            >
                              Active
                            </th>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Geometry
                            </th>
                            <th
                              style={{
                                textAlign: "right",
                                padding: "12px 8px",
                              }}
                            >
                              Created At
                            </th> 
                          </tr>                        
                        </thead>
                        <tbody>
                          {waypoints.length === 0 && (
                            <tr>
                              <td
                                colSpan={5}
                                style={{
                                  padding: "36px 8px",
                                  textAlign: "center",
                                  color: "#94a3b8",
                                }}
                              >
                                <div
                                  style={{
                                    fontWeight: 700,
                                    color: "#0f172a",
                                    marginBottom: 6,
                                  }}
                                >
                                  No Waypoints Found
                                </div>
                                <div>Try creating new waypoints.</div>
                              </td>
                            </tr>
                          )}
                          {waypoints.map((wp) => (
                            <tr
                              key={wp.id}
                              onClick={() => handleSelectWaypoint(wp.id)}
                              style={{
                                cursor: "pointer",
                                background:
                                  selectedWaypointId === wp.id
                                    ? "rgba(3,48,80,0.04)"
                                    : "transparent",
                              }}
                            >
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  fontWeight: 700,
                                }}
                              >
                                {wp.name}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  color: "#6b7280",
                                }}
                              >
                                {wp.category}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "center",
                                }}
                              >
                                {wp.active ? "Yes" : "No"}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  color: "#6b7280",
                                }}
                              >
                                {wp.geom}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "right",
                                }}
                              >
                                {wp.createdAt}
                              </td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                  </div>
                </div>
              )}

              {/* Users page: search, create button and table similar to your screenshot */}
              {rightPage === "users" && (
                <div
                  style={{ display: "flex", flexDirection: "column", gap: 12 }}
                >
                  <div
                    style={{
                      display: "flex",
                      justifyContent: "space-between",
                      alignItems: "center",
                    }}
                  >
                    <div
                      style={{ display: "flex", alignItems: "center", gap: 8 }}
                    >
                      <label
                        style={{
                          fontSize: 13,
                          color: "#475569",
                          fontWeight: 600,
                        }}
                      >
                        Search User By
                      </label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Email</option>
                      </select>
                      <input
                        placeholder="Search user..."
                        style={{
                          padding: "8px 10px",
                          borderRadius: 8,
                          border: "1px solid #e6eef2",
                          minWidth: 300,
                        }}
                      />
                    </div>
                    <button
                      style={{
                        background: "#0b74d1",
                        color: "#fff",
                        padding: "10px 14px",
                        borderRadius: 8,
                        border: "none",
                        cursor: "pointer",
                      }}
                    >
                      + Create New User
                    </button>
                  </div>

                  <div
                    style={{
                      background: "#fff",
                      borderRadius: 8,
                      overflow: "hidden",
                      boxShadow: "0 1px 3px rgba(2,6,23,0.06)",
                    }}
                  >
                    <div
                      style={{
                        padding: "12px 16px",
                        borderBottom: "1px solid #eef2f6",
                        display: "flex",
                        alignItems: "center",
                        gap: 12,
                      }}
                    >
                      <div style={{ fontWeight: 800, fontSize: 18 }}>Users</div>
                      <div
                        style={{
                          marginLeft: "auto",
                          display: "flex",
                          alignItems: "center",
                          gap: 12,
                        }}
                      >
                        {usersError && (
                          <span style={{ color: "#dc2626", fontSize: 13 }}>
                            {usersError}
                          </span>
                        )}
                        <button
                          onClick={loadUsers}
                          disabled={usersLoading}
                          style={{
                            padding: "6px 12px",
                            borderRadius: 6,
                            border: "1px solid #dbe3ea",
                            background: usersLoading ? "#f1f5f9" : "#fff",
                            color: "#0f172a",
                            cursor: usersLoading ? "not-allowed" : "pointer",
                          }}
                        >
                          {usersLoading ? "Refreshing..." : "Refresh"}
                        </button>
                      </div>
                    </div>

                    <div style={{ padding: "8px 16px" }}>
                      <table
                        style={{ width: "100%", borderCollapse: "collapse" }}
                      >
                        <thead
                          style={{
                            background: "#fafafa",
                            color: "#475569",
                            fontSize: 13,
                          }}
                        >
                          <tr>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Name
                            </th>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Email
                            </th>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Role
                            </th>
                            <th
                              style={{
                                textAlign: "center",
                                padding: "12px 8px",
                              }}
                            >
                              Status
                            </th>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Created By
                            </th>
                            <th
                              style={{
                                textAlign: "right",
                                padding: "12px 8px",
                              }}
                            >
                              Created At
                            </th>
                            <th
                              style={{
                                textAlign: "center",
                                padding: "12px 8px",
                                width: 48,
                              }}
                            ></th>
                          </tr>
                        </thead>
                        <tbody>
                          {users.map((u) => (
                            <tr
                              key={u.id}
                              onClick={() => setSelectedUserId(u.id)}
                              style={{
                                cursor: "pointer",
                                background:
                                  selectedUserId === u.id
                                    ? "rgba(3,48,80,0.04)"
                                    : "transparent",
                              }}
                            >
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  fontWeight: 700,
                                }}
                              >
                                {u.name}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                                                   color: "#6b7280",
                                }}
                              >
                                {u.email}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                }}
                              >
                                {u.role}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "center",
                                }}
                              >
                                <span
                                  style={{
                                    background: "#bbf7d0",
                                    color: "#065f46",
                                    padding: "4px 8px",
                                    borderRadius: 8,
                                    fontSize: 12,
                                  }}
                                >
                                  {u.status}
                                </span>
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  color: "#6b7280",
                                }}
                              >
                                {u.createdBy}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "right",
                                  color: "#6b7280",
                                }}
                              >
                                {u.createdAt}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "center",
                                }}
                              >
                                <button
                                  title="Delete user"
                                  onClick={(e) => {
                                    e.stopPropagation(); /* placeholder */
                                  }}
                                  style={{
                                    background: "transparent",
                                    border: "none",
                                    cursor: "pointer",
                                    color: "#9ca3af",
                                  }}
                                >
                                  <FaTrash />
                                </button>
                              </td>
                            </tr>
                          ))}
                          {users.length === 0 && !usersLoading && (
                            <tr>
                              <td
                                colSpan={7}
                                style={{
                                  padding: "16px 8px",
                                  textAlign: "center",
                                  color: "#94a3b8",
                                }}
                              >
                                No users found.
                              </td>
                            </tr>
                          )}
                          {usersLoading && (
                            <tr>
                              <td
                                colSpan={7}
                                style={{
                                  padding: "16px 8px",
                                  textAlign: "center",
                                  color: "#94a3b8",
                                }}
                              >
                                Loading users...
                              </td>
                            </tr>
                          )}
                        </tbody>
                      </table>
                    </div>

                    <div
                      style={{
                        display: "flex",
                        alignItems: "center",
                        gap: 12,
                        justifyContent: "flex-end",
                        padding: "12px 16px",
                        borderTop: "1px solid #eef2f6",
                      }}
                    >
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>
                        Rows per page: 5
                      </div>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>
                        1–{users.length} of {users.length}
                      </div>
                      <div style={{ marginLeft: 8, display: "flex", gap: 8 }}>
                        <button
                          style={{
                            padding: "8px 12px",
                            borderRadius: 8,
                            border: "1px solid #e6eef2",
                            background: "#fff",
                            color: "#6b7280",
                          }}
                          disabled
                        >
                          Edit
                        </button>
                        <button
                          style={{
                            padding: "8px 12px",
                            borderRadius: 8,
                            border: "1px solid #e6eef2",
                            background: "#fff",
                            color: "#6b7280",
                          }}
                          disabled={!selectedUserId || userActionLoading}
                          onClick={handleResetUserPassword}
                        >
                          {userActionLoading ? "Resetting..." : "Reset Password"}
                        </button>
                      </div>
                    </div>
                  </div>
                </div>
              )}

              {/* Missions page: search, create button and table */}
              {rightPage === "missions" && (
                <div
                  style={{ display: "flex", flexDirection: "column", gap: 12 }}
                >
                  <div
                    style={{
                      display: "flex",
                      justifyContent: "space-between",
                      alignItems: "center",
                    }}
                  >
                    <div
                      style={{ display: "flex", alignItems: "center", gap: 8 }}
                    >
                      <label
                        style={{
                          fontSize: 13,
                          color: "#475569",
                          fontWeight: 600,
                        }}
                      >
                        Search Mission By
                      </label>
                      <select style={{ padding: "6px 8px" }}>
                        <option>Name</option>
                        <option>Owner</option>
                        <option>Status</option>
                      </select>
                      <input
                        placeholder="Search mission..."
                        style={{
                          padding: "8px 10px",
                          borderRadius: 8,
                          border: "1px solid #e6eef2",
                          minWidth: 220,
                        }}
                      />
                    </div>

                    <button
                      style={{
                        background: "#0b74d1",
                        color: "#fff",
                        padding: "10px 14px",
                        borderRadius: 8,
                        border: "none",
                        cursor: "pointer",
                      }}
                      onClick={() => setMissionFormOpen((v) => !v)}
                    >
                      + Create Mission
                    </button>
                  </div>

                  {missionFormOpen && (
                    <div className="mission-form">
                      <div className="zone-form-row">
                        <label>
                          Name
                          <input
                            value={missionForm.name}
                            onChange={(e) =>
                              setMissionForm((prev) => ({
                                ...prev,
                                name: e.target.value,
                              }))
                            }
                            placeholder="Mission name"
                          />
                        </label>
                        <label>
                          Owner
                          <input
                            value={missionForm.owner}
                            onChange={(e) =>
                              setMissionForm((prev) => ({
                                ...prev,
                                owner: e.target.value,
                              }))
                            }
                            placeholder="Owner or department"
                          />
                        </label>
                        <label>
                          Status
                          <select
                            value={missionForm.status}
                            onChange={(e) =>
                              setMissionForm((prev) => ({
                                ...prev,
                                status: e.target.value,
                              }))
                            }
                          >
                            <option value="Draft">Draft</option>
                            <option value="Scheduled">Scheduled</option>
                            <option value="In Progress">In Progress</option>
                            <option value="Completed">Completed</option>
                          </select>
                        </label>
                      </div>
                      <label>
                        Notes
                        <textarea
                          value={missionForm.notes}
                          onChange={(e) =>
                            setMissionForm((prev) => ({
                              ...prev,
                              notes: e.target.value,
                            }))
                          }
                          placeholder="Mission objectives, constraints, etc."
                          className="mission-notes"
                        />
                      </label>
                      <div
                        className="zone-form-actions"
                        style={{ marginLeft: "auto" }}
                      >
                        <button
                          className="ghost-btn"
                          type="button"
                          onClick={() => {
                            setMissionFormOpen(false);
                            setMissionForm({
                              name: "",
                              owner: "",
                              status: "Draft",
                              notes: "",
                            });
                          }}
                        >
                          Cancel
                        </button>
                        <button
                          className="primary-btn"
                          type="button"
                          onClick={async () => {
                            if (!missionForm.name.trim()) {
                              toast.error("Mission name required");
                              return;
                            }
                            if (!missionForm.owner.trim()) {
                              toast.error("Mission owner required");
                              return;
                            }
                            const payload = {
                              name: missionForm.name.trim(),
                              owner: missionForm.owner.trim(),
                              status: missionForm.status,
                              notes: missionForm.notes,
                            };
                            try {
                          const response = await missionRequest("/missions", {
                            method: "POST",
                            body: JSON.stringify(payload),
                          });
                              const createdMission = response.item || payload;
                              setMissions((prev) => [createdMission, ...prev]);
                              setSelectedMissionId(createdMission.id);
                              setMissionForm({
                                name: "",
                                owner: "",
                                status: "Draft",
                                notes: "",
                              });
                              setMissionFormOpen(false);
                              toast.success("Mission saved");
                            } catch (error) {
                              console.error("Mission create error", error);
                              toast.error(
                                error.message || "Failed to create mission",
                              );
                            }
                          }}
                        >
                          Save Mission
                        </button>
                      </div>
                    </div>
                  )}

                  <div
                    style={{ borderTop: "1px solid #e6eef2", marginTop: 4 }}
                  />

                  <div
                    style={{
                      background: "#fff",
                      borderRadius: 8,
                      boxShadow: "0 1px 3px rgba(2,6,23,0.06)",
                      overflow: "hidden",
                    }}
                  >
                    <div
                      style={{
                        padding: "12px 16px",
                        borderBottom: "1px solid #eef2f6",
                        display: "flex",
                        alignItems: "center",
                        gap: 12,
                      }}
                    >
                      <div
                        style={{ flex: 1, fontWeight: 700, color: "#0f172a" }}
                      >
                        Missions
                      </div>
                      <div style={{ color: "#94a3b8", fontSize: 13 }}>
                        Rows per page: 10
                      </div>
                    </div>
                    <div style={{ padding: "8px 16px" }}>
                      <table
                        style={{ width: "100%", borderCollapse: "collapse" }}
                      >
                        <thead
                          style={{
                            background: "#fafafa",
                            color: "#475569",
                            fontSize: 13,
                          }}
                        >
                          <tr>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Name
                            </th>
                            <th
                              style={{ textAlign: "left", padding: "12px 8px" }}
                            >
                              Owner
                            </th>
                            <th
                              style={{
                                textAlign: "center",
                                padding: "12px 8px",
                              }}
                            >
                              Status
                            </th>
                            <th
                              style={{
                                textAlign: "right",
                                padding: "12px 8px",
                              }}
                            >
                              Created At
                            </th>
                          </tr>
                        </thead>
                        <tbody>
                          {missions.map((m) => (
                            <tr
                              key={m.id}
                              onClick={() => handleSelectMission(m.id)}
                              style={{
                                cursor: "pointer",
                                background:
                                  selectedMissionId === m.id
                                    ? "rgba(3,48,80,0.04)"
                                    : "transparent",
                              }}
                            >
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  fontWeight: 700,
                                }}
                              >
                                {m.name}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  color: "#6b7280",
                                }}
                              >
                                {m.owner}
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "center",
                                }}
                              >
                                <span
                                  style={{
                                    background:
                                      m.status === "Completed"
                                        ? "#bbf7d0"
                                        : "#fee2e2",
                                    color:
                                      m.status === "Completed"
                                        ? "#065f46"
                                        : "#9b1b1b",
                                    padding: "4px 8px",
                                    borderRadius: 8,
                                    fontSize: 12,
                                  }}
                                >
                                  {m.status}
                                </span>
                              </td>
                              <td
                                style={{
                                  padding: "12px 8px",
                                  borderBottom: "1px solid #eef2f6",
                                  textAlign: "right",
                                  color: "#6b7280",
                                }}
                              >
                                {m.createdAt}
                              </td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                  </div>

                  {/* mission details */}
                  <div
                    style={{
                      background: "#fff",
                      borderRadius: 8,
                      padding: 12,
                      boxShadow: "0 1px 3px rgba(2,6,23,0.06)",
                    }}
                  >
                    {selectedMissionId ? (
                      (() => {
                        const m = missions.find(
                          (x) => x.id === selectedMissionId,
                        );
                        if (!m)
                          return (
                            <div style={{ color: "#94a3b8" }}>
                              Mission not found.
                            </div>
                          );


                        return (
                          <div>
                            <div style={{ fontWeight: 800, fontSize: 16 }}>
                              {m.name}
                            </div>
                            <div style={{ marginTop: 8, color: "#6b7280" }}>
                              <strong>Owner:</strong> {m.owner}
                            </div>
                            <div style={{ marginTop: 6, color: "#6b7280" }}>
                              <strong>Status:</strong> {m.status}
                            </div>
                            <div style={{ marginTop: 6, color: "#6b7280" }}>
                              <strong>Created At:</strong> {m.createdAt}
                            </div>
                            <div style={{ marginTop: 10, color: "#475569" }}>
                              {m.notes || "No notes."}
                            </div>
                            <div style={{ marginTop: 14, display: "flex", gap: 8 }}>
                              <button
                                className="primary-btn"
                                onClick={() => handleInitiateMission(m.id)}
                                disabled={missionActionLoading}
                              >
                                {missionActionLoading ? "Dispatching..." : "Initiate Mission"}
                              </button>
                            </div>
                          </div>
                        );
                      })()
                    ) : (
                      <div style={{ color: "#94a3b8" }}>
                        Select a mission to see details.
                      </div>
                    )}
                  </div>
                </div>
              )}

              {rightPage === "analytics" && (
                <div className="analytics-pane">
                  <div className="analytics-kpis">
                    {analyticsSummary.map((card) => (
                      <div key={card.label} className="kpi-card">
                        <span className="kpi-label">{card.label}</span>
                        <strong>{card.value}</strong>
                        <span className="kpi-trend">{card.trend}</span>
                      </div>
                    ))}
                  </div>
                  <div className="analytics-chart-card">
                    <div className="stats-card-header">
                      <div>
                        <h4>Cycle Throughput</h4>
                        <p>Rolling seven-day view</p>
                      </div>
                    </div>
                    <svg
                      width={analyticsChartSize.width}
                      height={analyticsChartSize.height}
                      className="analytics-chart"
                    >
                      <polyline
                        points={analyticsPath}
                        fill="none"
                        strokeWidth="3"
                        className="analytics-line"
                      />
                    </svg>
                  </div>
                  <div className="analytics-alerts">
                    {analyticsAlerts.map((alert) => (
                      <div key={alert.id} className="alert-card">
                        <strong>{alert.title}</strong>
                        <p>{alert.detail}</p>
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {rightPage === "diagnostics" && (
                <div className="diagnostics-pane">
                  {diagnosticsPanels.map((panel) => (
                    <div key={panel.id} className="diag-card">
                      <h4>{panel.title}</h4>
                      <div className="diag-value">{panel.value}</div>
                      <span className="diag-status">{panel.status}</span>
                      <p>{panel.detail}</p>
                    </div>
                  ))}
                </div>
              )}

              {rightPage === "bridge" && (
                <div className="analytics-pane">
                  <div className="analytics-kpis" style={{ gap: 12 }}>
                    <div className="kpi-card" style={{ flex: 1 }}>
                      <span className="kpi-label">AMR Bridge</span>
                      <strong>{bridgeStatus.connected ? "Connected" : "Offline"}</strong>
                      <span className="kpi-trend">
                        {bridgeStatus.endpoint || "—"}{" "}
                        {bridgeStatus.error ? ` · ${bridgeStatus.error}` : ""}
                      </span>
                    </div>
                  </div>
                  <div className="analytics-chart-card">
                    <div className="stats-card-header">
                      <div>
                        <h4>Bridge Layers</h4>
                        <p>Expected IDs by layer</p>
                      </div>
                    </div>
                    <table style={{ width: "100%", borderCollapse: "collapse" }}>
                      <thead>
                        <tr style={{ textAlign: "left", borderBottom: "1px solid #e5e7eb" }}>
                          <th style={{ padding: "10px 8px" }}>Layer</th>
                          <th style={{ padding: "10px 8px" }}>Purpose</th>
                          <th style={{ padding: "10px 8px" }}>API IDs</th>
                        </tr>
                      </thead>
                      <tbody>
                        {bridgeApiLayers.map((row) => (
                          <tr key={row.layer} style={{ borderBottom: "1px solid #f1f5f9" }}>
                            <td style={{ padding: "10px 8px", fontWeight: 700 }}>{row.layer}</td>
                            <td style={{ padding: "10px 8px", color: "#475569" }}>{row.purpose}</td>
                            <td style={{ padding: "10px 8px", color: "#0f172a" }}>{row.ids}</td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                </div>
              )}

              {rightPage === "camera" && (
                <div className="pane-wrapper">
                  <div className="pane-controls">
                    <h4>AMR Camera</h4>
                    <input
                      type="text"
                      value={cameraUrl}
                      onChange={(e) => setCameraUrl(e.target.value)}
                      placeholder="http://camera-host/stream.mjpg"
                      className="pane-input"
                      aria-label="Camera stream URL"
                    />
                    <div className="pane-meta">
                      {cameraUrl ? "Stream configured" : "No URL set"}
                      {cameraUrl ? ` · ${cameraUrl}` : ""}
                    </div>
                  </div>
                  <div className="pane-card">
                    {cameraUrl ? (
                      cameraUrl.toLowerCase().includes("mjpg") ||
                      cameraUrl.toLowerCase().includes("multipart") ||
                      cameraUrl.toLowerCase().includes("stream") ? (
                        <img
                          key={cameraUrl}
                          src={cameraUrl}
                          alt="AMR camera stream"
                          style={{ width: "100%", maxHeight: 480, objectFit: "contain", background: "#000" }}
                        />
                      ) : (
                        <video
                          key={cameraUrl}
                          src={cameraUrl}
                          controls
                          autoPlay
                          muted
                          style={{ width: "100%", maxHeight: 480, background: "#000" }}
                        >
                          Your browser does not support the video tag.
                        </video>
                      )
                    ) : (
                      <div style={{ color: "#94a3b8" }}>
                        Enter a camera stream URL to view video.
                      </div>
                    )}
                  </div>
                </div>
              )}

              {rightPage === "logs" && (
                <div className="logs-pane">
                  <table>
                    <thead>
                      <tr>
                        <th>Time</th>
                        <th>Component</th>
                        <th>Message</th>
                        <th>Level</th>
                      </tr>
                    </thead>
                    <tbody>
                      {logEvents.map((event) => (
                        <tr key={event.id}>
                          <td>{event.ts}</td>
                          <td>{event.system}</td>
                          <td>{event.message}</td>
                          <td>
                            <span className={`log-pill ${event.level}`}>
                              {event.level}
                            </span>
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              )}

              {rightPage === "mission-logs" && (
                <div className="mission-log-pane">
                  {missionHistory.map((entry) => (
                    <div key={entry.id} className="timeline-card">
                      <div className="timeline-header">
                        <strong>{entry.mission}</strong>
                        <span>{entry.window}</span>
                      </div>
                      <div
                        className={`timeline-status ${entry.outcome === "Completed" ? "success" : "warn"}`}
                      >
                        {entry.outcome}
                      </div>
                      <p>{entry.notes}</p>
                    </div>
                  ))}
                </div>
              )}

              {rightPage === "robot-bags" && (
                <div className="bags-pane">
                  <table>
                    <thead>
                      <tr>
                        <th>Filename</th>
                        <th>Duration</th>
                        <th>Size</th>
                        <th>Status</th>
                        <th></th>
                      </tr>
                    </thead>
                    <tbody>
                      {bagFiles.map((bag) => (
                        <tr key={bag.id}>
                          <td>{bag.name}</td>
                          <td>{bag.duration}</td>
                          <td>{bag.size}</td>
                          <td>
                            <span
                              className={`bag-pill ${bag.status.toLowerCase()}`}
                            >
                              {bag.status}
                            </span>
                          </td>
                          <td>
                            <button className="ghost-btn">Download</button>
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              )}

              {rightPage === "robot-settings" && (
                <div className="settings-pane">
                  {Object.entries(robotSettingsState).map(([key, value]) => (
                    <label key={key} className="toggle-row">
                      <input
                        type="checkbox"
                        checked={value}
                        onChange={() => toggleRobotSetting(key)}
                      />
                      <div>
                        <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                        <p>{value ? "Enabled" : "Disabled"}</p>
                      </div>
                    </label>
                  ))}
                </div>
              )}

              {rightPage === "account" && (
                <div className="account-pane">
                  <label>
                    Name
                    <input
                      value={accountProfile.fullName}
                      onChange={(e) =>
                        handleAccountChange("fullName", e.target.value)
                      }
                    />
                  </label>
                  <label>
                    Email
                    <input
                      value={accountProfile.email}
                      onChange={(e) =>
                        handleAccountChange("email", e.target.value)
                      }
                    />
                  </label>
                  <label>
                    Team
                    <input
                      value={accountProfile.team}
                      onChange={(e) =>
                        handleAccountChange("team", e.target.value)
                      }
                    />
                  </label>
                  <label>
                    Shift Window
                    <input
                      value={accountProfile.shift}
                      onChange={(e) =>
                        handleAccountChange("shift", e.target.value)
                      }
                    />
                  </label>
                  {profileError && (
                    <p style={{ color: "red" }}>{profileError}</p>
                  )}
                  {profileSuccess && (
                    <p style={{ color: "green" }}>{profileSuccess}</p>
                  )}
                  <button
                    className="primary-btn"
                    type="button"
                    onClick={handleSaveProfile}
                    disabled={profileSaving}
                  >
                    {profileSaving ? "Saving..." : "Save Profile"}
                  </button>
                </div>
              )}

              {rightPage === "appearance" && (
                <div className="appearance-pane">
                  {[
                    { id: "light", label: "Light" },
                    { id: "dark", label: "Dark" },
                    { id: "system", label: "Match System" },
                  ].map((theme) => (
                    <button
                      key={theme.id}
                      className={`theme-card ${selectedTheme === theme.id ? "active" : ""}`}
                      onClick={() => setSelectedTheme(theme.id)}
                    >
                      <strong>{theme.label}</strong>
                      <span>
                        {selectedTheme === theme.id ? "Selected" : "Use theme"}
                      </span>
                    </button>
                  ))}
                </div>
              )}

              {rightPage === "security" && (
                <div className="security-pane">
                  <div className="security-toggles">
                    {Object.entries(securityPreferences).map(([key, value]) => (
                      <label key={key} className="toggle-row">
                        <input
                          type="checkbox"
                          checked={value}
                          onChange={() => toggleSecurityPref(key)}
                        />
                        <div>
                          <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                          <p>{value ? "Enabled" : "Disabled"}</p>
                        </div>
                      </label>
                    ))}
                  </div>
                  <table className="security-table">
                    <thead>
                      <tr>
                        <th>Time</th>
                        <th>Actor</th>
                        <th>Action</th>
                        <th>Context</th>
                      </tr>
                    </thead>
                    <tbody>
                      {securityEvents.map((evt) => (
                        <tr key={evt.id}>
                          <td>{evt.ts}</td>
                          <td>{evt.actor}</td>
                          <td>{evt.action}</td>
                          <td>{evt.context}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              )}

              {rightPage === "integrations" && (
                <div className="integrations-pane">
                  {integrationItems.map((integration) => (
                    <div key={integration.id} className="integration-card">
                      <div>
                        <strong>{integration.name}</strong>
                        <p>{integration.description}</p>
                      </div>
                      <div className="integration-meta">
                        <span
                          className={`integration-status ${integration.status === "Connected" ? "ok" : "off"}`}
                        >
                          {integration.status}
                        </span>
                        <button
                          className="ghost-btn"
                          onClick={() =>
                            toggleIntegrationStatus(integration.id)
                          }
                        >
                          {integration.status === "Connected"
                            ? "Disconnect"
                            : "Connect"}
                        </button>
                      </div>
                    </div>
                  ))}
                </div>
              )}
              {rightPage === "stats" && (
                <div className="stats-pane">
                  <div className="stats-status-row">
                    {statsLoading && (
                      <span className="stats-status">
                        Refreshing telemetry…
                      </span>
                    )}
                    {statsError && (
                      <span className="stats-error">{statsError}</span>
                    )}
                  </div>
                  <div className="stats-summary">
                    <div className="stats-card">
                      <span className="stats-label">Total Moving Distance</span>
                      <h3>{overview.totalKm.toFixed(1)} km</h3>
                      <p>+{overviewDeltaLabel} km vs previous day</p>
                    </div>
                    <div className="stats-card">
                      <span className="stats-label">Missions Completed</span>
                      <h3>{overview.missionsCompleted}</h3>
                      <p>{overview.missionSuccessRate}% success over 7 days</p>
                    </div>
                    <div className="stats-card">
                      <span className="stats-label">Average Speed</span>
                      <h3>{overview.avgSpeed.toFixed(1)} m/s</h3>
                      <p>Within safe corridor</p>
                    </div>
                    <div className="stats-card">
                      <span className="stats-label">Operating Hours</span>
                      <h3>{overview.operatingHours} h</h3>
                      <p>Last maintenance at 300 h</p>
                    </div>
                  </div>

                  <div className="movement-card">
                    <div className="stats-card-header">
                      <div>
                        <h4>Monthly Movement</h4>
                        <p>Distance travelled per month</p>
                      </div>
                    </div>
                    <div className="movement-summary">
                      <div>
                        <span className="movement-label">Total</span>
                        <strong>{totalMonthlyKm.toFixed(1)} km</strong>
                      </div>
                      <div>
                        <span className="movement-label">Average / month</span>
                        <strong>{avgMonthlyKm} km</strong>
                      </div>
                    </div>
                    <div className="movement-bars">
                      {monthlyMovement.map((entry) => (
                        <div key={entry.month} className="movement-bar">
                          <div
                            className="movement-bar-fill"
                            style={{
                              height: `${(entry.km / (monthlyMaxKm || 1)) * 100}%`,
                            }}
                            title={`${entry.km} km`}
                          />
                          <span>{entry.month}</span>
                        </div>
                      ))}
                    </div>
                  </div>

                  <div className="stats-grid">
                    <div className="stats-chart-card">
                      <div className="stats-card-header">
                        <div>
                          <h4>Battery Voltage & Power</h4>
                          <p>Live pack telemetry</p>
                        </div>
                        <div className="stats-legend">
                          <span className="legend-dot voltage" /> Voltage
                          <span className="legend-dot power" /> Power
                        </div>
                      </div>
                      <div className="line-chart">
                        <svg
                          width={lineChartSize.width}
                          height={lineChartSize.height}
                          role="img"
                          aria-label="Battery voltage and power line plot"
                        >
                          {[0.25, 0.5, 0.75, 1].map((ratio) => (
                            <line
                              key={ratio}
                              x1="0"
                              x2={lineChartSize.width}
                              y1={lineChartSize.height * ratio}
                              y2={lineChartSize.height * ratio}
                              className="chart-grid-line"
                            />
                          ))}
                          <polyline
                            points={batteryVoltagePath}
                            className="line-voltage"
                            fill="none"
                            strokeWidth="3"
                          />
                          <polyline
                            points={batteryPowerPath}
                            className="line-power"
                            fill="none"
                            strokeWidth="2"
                          />
                        </svg>
                        <div className="chart-x-axis">
                          {batterySeries.map((point) => (
                            <span key={point.time}>{point.time}</span>
                          ))}
                        </div>
                      </div>
                    </div>

                    <div className="turn-card">
                      <div className="stats-card-header">
                        <div>
                          <h4>Turn Distribution</h4>
                          <p>Number of left/right turns this shift</p>
                        </div>
                      </div>
                      <div className="turn-count-row">
                        <span>Left turns</span>
                        <strong>{turns.left}</strong>
                      </div>
                      <div className="turn-bar">
                        <div
                          className="turn-bar-left"
                          style={{ width: `${leftTurnPercent}%` }}
                        />
                      </div>
                      <div className="turn-count-row">
                        <span>Right turns</span>
                        <strong>{turns.right}</strong>
                      </div>
                      <div className="turn-bar">
                        <div
                          className="turn-bar-right"
                          style={{ width: `${rightTurnPercent}%` }}
                        />
                      </div>
                      <div className="turn-footer">
                        Left {leftTurnPercent}% · Right {rightTurnPercent}%
                      </div>
                    </div>
                  </div>

                  <div className="trend-card">
                    <div className="stats-card-header">
                      <div>
                        <h4>Mission Trend</h4>
                        <p>Completion vs incidents</p>
                      </div>
                    </div>
                    <table>
                      <thead>
                        <tr>
                          <th>Day</th>
                          <th>Completed</th>
                          <th>Incidents</th>
                        </tr>
                      </thead>
                      <tbody>
                        {missionTrend.map((row) => (
                          <tr key={row.label}>
                            <td>{row.label}</td>
                            <td>{row.completed}</td>
                            <td>{row.incidents}</td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                </div>
              )}
              {/* Chat page */}
              {rightPage === "chat" && (
                <div className="chat-pane">
                  <div className="chat-header">
                    <div>
                      <h2>Assistant Link</h2>
                      <p>
                        Last sync at{" "}
                        {latestMessage
                          ? formatTimestamp(latestMessage.timestamp)
                          : "--:--"}{" "}
                        ·{" "}
                        {
                          chatMessages.filter((m) => m.sender === "human")
                            .length
                        }{" "}
                        operator prompts today
                      </p>
                    </div>
                    <div className="chat-status-pill">Robot Online</div>
                  </div>
                  <div className="chat-subheader">
                    <span>Channel: Operations Support</span>
                    <span>
                      Voice input {isRecording ? "recording…" : "idle"}
                    </span>
                  </div>

                  <div className="chat-messages" ref={chatContainerRef}>
                    {chatMessages.map((message) => {
                      const isRobot = message.sender === "robot";
                      const meta = message.metadata || {};
                      return (
                        <div
                          key={message.id}
                          className={`chat-row ${isRobot ? "robot" : "human"}`}
                        >
                          <div
                            className={`chat-bubble ${isRobot ? "robot" : "human"}`}
                          >
                            <div className="chat-meta">
                              <span>{isRobot ? "Robot" : "You"}</span>
                              <span>{formatTimestamp(message.timestamp)}</span>
                              {message.status && (
                                <span className="chat-status">{message.status}</span>
                              )}
                            </div>
                            <p>{message.text}</p>
                            {meta.intent && (
                              <div className="chat-intent-chip">
                                Intent: {meta.intent}
                                {meta.source ? ` (${meta.source})` : ""}
                              </div>
                            )}
                            {Array.isArray(meta.actions) && meta.actions.length > 0 && (
                              <div className="chat-actions-list">
                                {meta.actions.map((act, idx) => (
                                  <span key={idx} className="chat-chip">
                                    {act.description || act.action || "Action"}
                                  </span>
                                ))}
                              </div>
                            )}
                          </div>
                        </div>
                      );
                    })}
                    {isTyping && (
                      <div className="chat-row robot">
                        <div className="chat-bubble robot typing">
                          <span className="typing-dot" />
                          <span className="typing-dot" />
                          <span className="typing-dot" />
                        </div>
                      </div>
                    )}
                  </div>

                  <div className="chat-quick-replies">
                    {chatQuickPrompts.map((prompt) => (
                      <button
                        key={prompt}
                        type="button"
                        className="chat-chip"
                        onClick={() => handleSuggestionClick(prompt)}
                      >
                        {prompt}
                      </button>
                    ))}
                  </div>

                  <div className="chat-input-row">
                    <textarea
                      value={chatInput}
                      onChange={(e) => setChatInput(e.target.value)}
                      onKeyPress={handleKeyPress}
                      placeholder="Type a request or update…"
                      className="chat-textarea"
                      rows={1}
                    />
                    <button
                      onClick={handleMicClick}
                      type="button"
                      className={`chat-icon-btn ${isRecording ? "recording" : ""}`}
                      title={isRecording ? "Stop recording" : "Start recording"}
                    >
                      <FaMicrophone />
                    </button>
                    <button
                      onClick={() => handleSendMessage()}
                      disabled={!chatInput.trim()}
                      type="button"
                      className="chat-send-btn"
                      title="Send message"
                    >
                      <FaPaperPlane />
                    </button>
                  </div>
                  <div className="chat-hint">
                    Press Enter to send · Shift + Enter for a newline
                  </div>
                </div>
              )}
              {/* fallback */}
              {![
                "maps",
                "zones",
                "waypoints",
                "missions",
                "users",
                "analytics",
                "diagnostics",
                "logs",
                "mission-logs",
                "robot-bags",
                "robot-settings",
                "account",
                "appearance",
                "security",
                "integrations",
                "chat",
              ].includes(rightPage) && <div>{rightPage}</div>}
            </div>
          </aside>
        )}

        <main className={`map-area ${minimizedMain ? "minimized" : ""}`}>
          {/* Map / Workspace placeholder - this is the element we fullscreen */}
          <div
            ref={mapRef}
            className="map-ref"
            /* ref kept on outer container so fullscreen targets the whole map area */
          >
            {/* inner content that will zoom — click handler here so only map-content scales */}
            <div
              className={`map-content`}
              onClick={toggleMapZoom}
              role="button"
              tabIndex={0}
              onKeyDown={(e) => {
                if (e.key === "Enter" || e.key === " ") toggleMapZoom();
              }}
              style={{
                width: "100%",
                height: "100%",
                display: "flex",
                alignItems: "stretch",
                justifyContent: "stretch",
                transform: `scale(${1 + zoomLevel})`,
                transition: "transform 280ms ease",
                cursor: zoomLevel > 0 ? "zoom-out" : "zoom-in",
              }}
            >
              {/* intentionally render nothing in the center */}
              <div style={{ width: "100%", height: "100%" }} aria-hidden="true" />
            </div>

            <div className="map-overlays">
              <div className="right-controls" onClick={(e) => e.stopPropagation()}>
                <button
                  className="control-btn"
                  title="Zoom In"
                  onClick={(e) => { e.stopPropagation(); zoomIn(e); }}
                  aria-label="Zoom in"
                  type="button"
                >
                  ＋
                </button>
                <button
                  className="control-btn"
                  title="Zoom Out"
                  onClick={(e) => { e.stopPropagation(); zoomOut(e); }}
                  aria-label="Zoom out"
                  type="button"
                >
                  −
                </button>
              </div>
              <div className={`joystick-overlay ${minimizedMain ? "minimized" : ""}`}>
                <JoyStick width={140} height={140} onMove={handleJoystickMove} />
              </div>
            </div>
          </div>
        </main>
      </div>

      {batteryModalOpen && (
        <div
          className="modal-backdrop"
          onClick={() => setBatteryModalOpen(false)}
        >
          <div className="battery-modal" onClick={(e) => e.stopPropagation()}>
            <div className="battery-modal-header">
              <h3>Battery Status</h3>
              <button
                className="right-pane-close"
                type="button"
                onClick={() => setBatteryModalOpen(false)}
                aria-label="Close battery modal"
              >
                ✕
              </button>
            </div>
            <div className="battery-modal-body">
              <div className="battery-modal-grid">
                <div>
                  <span className="battery-label">Pack Voltage</span>
                  <strong>{batteryStatus.packVoltage} V</strong>
                </div>
                <div>
                  <span className="battery-label">Pack Current</span>
                  <strong>{batteryStatus.packCurrent} A</strong>
                </div>
                <div>
                  <span className="battery-label">State of Charge</span>
                  <strong>{batteryStatus.stateOfCharge}%</strong>
                </div>
                <div>
                  <span className="battery-label">Temperature</span>
                  <strong>{batteryStatus.temperature}</strong>
                </div>
                <div>
                  <span className="battery-label">Cycles</span>
                  <strong>{batteryStatus.cycles}</strong>
                </div>
                <div>
                  <span className="battery-label">Health</span>
                  <strong>{batteryStatus.health}</strong>
                </div>
              </div>
              <div className="battery-cell-table">
                <table>
                  <thead>
                    <tr>
                      <th>Cell</th>
                      <th>Voltage</th>
                    </tr>
                  </thead>
                  <tbody>
                    {(batteryStatus.cells || []).map((cell) => (
                      <tr key={cell.id}>
                        <td>{cell.id}</td>
                        <td>{cell.voltage} V</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Lock overlay: shown when isLocked is true — covers full viewport including header */}
      {isLocked && (
        <div
          className="lock-overlay"
          role="button"
          aria-label="Locked overlay"
          tabIndex={0}
          onClick={(e) => {
            e.stopPropagation();
            showLockedAttemptToast();
          }}
          onMouseDown={(e) => {
            e.stopPropagation();
            showLockedAttemptToast();
          }}
          onKeyDown={(e) => {
            if (e.key === "Enter" || e.key === " ") {
              e.preventDefault();
              showLockedAttemptToast();
            }
          }}
        >
          {/* Visible hint so operators know how to unlock (Escape) */}
          <div className="unlock-hint" aria-hidden="true">
            Screen locked — press Esc to unlock
          </div>
        </div>
      )}

      {/* Fixed lock/unlock control that remains clickable when everything else is locked */}
      {isLocked && (
        <button
          type="button"
          className="lock-toggle-fixed"
          onClick={(e) => {
            e.stopPropagation();
            handleToggleLock();
          }}
          onMouseDown={(e) => e.preventDefault()}
          aria-pressed={isLocked}
          aria-label="Unlock console"
          title="Unlock console (Esc)"
        >
          {isLocked ? <FaLock /> : <FaUnlock />}
        </button>
      )}

      {mapModalOpen && (
          <div className="modal-backdrop" onClick={closeMapModal}>
            <div
              className="battery-modal"
              onClick={(e) => e.stopPropagation()}
              role="dialog"
              aria-modal="true"
              aria-label={mapModalMode === "edit" ? "Edit map" : "Create map"}
            >
              <div className="battery-modal-header">
                <h3>{mapModalMode === "edit" ? "Edit Map" : "Create Map"}</h3>
                <button
                  className="right-pane-close"
                  type="button"
                  onClick={closeMapModal}
                  aria-label="Close map modal"
                >
                  ✕
                </button>
              </div>
              <div className="battery-modal-body">
                <div style={{ display: "grid", gap: 10 }}>
                  <label>
                    Name
                    <input
                      value={mapForm.name}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, name: e.target.value }))
                      }
                      placeholder="Map name"
                    />
                  </label>
                  <label>
                    Created By
                    <input
                      value={mapForm.createdBy}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, createdBy: e.target.value }))
                      }
                      placeholder="Creator"
                    />
                  </label>
                  <label>
                    Status
                    <input
                      value={mapForm.status}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, status: e.target.value }))
                      }
                      placeholder="Active / Inactive / Draft"
                    />
                  </label>
                  <label>
                    Category
                    <input
                      value={mapForm.category}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, category: e.target.value }))
                      }
                      placeholder="Optional category"
                    />
                  </label>
                  <label>
                    Image (URL or path)
                    <input
                      value={mapForm.image}
                      onChange={(e) =>
                        setMapForm((prev) => ({ ...prev, image: e.target.value }))
                      }
                      placeholder="/images/maps/example.png"
                    />
                  </label>
                </div>

                <div style={{ display: "flex", gap: 8, justifyContent: "flex-end", marginTop: 12 }}>
                  <button
                    className="ghost-btn"
                    type="button"
                    onClick={closeMapModal}
                  >
                    Cancel
                  </button>
                  <button
                    className="primary-btn"
                    type="button"
                    onClick={saveMapFromForm}
                  >
                    {mapModalMode === "edit" ? "Save Changes" : "Create Map"}
                  </button>
                </div>
              </div>
            </div>
          </div>
        )}
    </div>
  );
};

export default MainPage;
