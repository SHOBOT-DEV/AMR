import React, { useRef, useEffect, useState, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import Sidebar from "../layout/SideBar";
import JoyStick from "./JoyStick";
import {
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
import "./MainPage.css";
import RightPane from "../layout/RightPane";
import Header from "../layout/Header";

const API_V1_BASE = `${API_BASE}/api/v1`;

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


  const MainPage = () => {
    const navigate = useNavigate();
    const mapRef = useRef(null);
    const layoutRef = useRef(null);
    const chatContainerRef = useRef(null);
    const mapImageInputRef = useRef(null); // hidden file input used by map modal

    // AMR Bridge state & connect helper (keeps same UI behavior as before)
    const [bridgeStatus, setBridgeStatus] = useState({
      connected: false,
      error: "",
      endpoint: "",
    });
    const startBridgeConnection = useCallback(() => {
      try {
        // show transient toast and simulate connection attempt (replace with real logic if available)
        toast.dismiss();
        toast.success("Retrying bridge connection…", {
          id: "bridge-conn",
          position: "top-right",
          duration: 1500,
        });
        setBridgeStatus({ connected: false, error: "Connecting…", endpoint: "" });
        // simulate async connect; in real app, call your bridge API here
        setTimeout(() => {
          setBridgeStatus({
            connected: true,
            error: "",
            endpoint: "amr://bridge.local",
          });
          toast.success("Connected to AMR bridge", { position: "top-right" });
        }, 900);
      } catch (err) {
        setBridgeStatus({ connected: false, error: err?.message || "Failed", endpoint: "" });
        toast.error("Bridge connection failed", { position: "top-right" });
      }
    }, []);

    // keep a local copy of locked state (Header will drive actual locking on the DOM)
    const [isLocked, setIsLocked] = useState(false);
    const showLockedAttemptToast = useCallback(() => {
      toast.dismiss("locked-attempt");
      toast.error("The screen is locked", { id: "locked-attempt" });
    }, []);



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

    // fullscreen / lock logic moved into Header (Header will operate on layoutRef)

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

    // Add zone search state
    const [zoneSearchField, setZoneSearchField] = useState("any");
    const [zoneSearchTerm, setZoneSearchTerm] = useState("");

    // waypoints data + selection
    const initialWaypoints = [
      {
        id: "wp1",
        mapId: "cfl_gf",
        name: "WP_A",
        category: "Nav",
        active: true,
        geom: "Point(12 34)",
        createdAt: "2025-11-17",
        notes: "First waypoint",
      },
      {
        id: "wp2",
        mapId: "cfl_gf",
        name: "WP_B",
        category: "Inspect",
        active: false,
        geom: "Point(98 76)",
        createdAt: "2025-11-17",
        notes: "Inspection point",
      },
      {
        id: "wp3",
        mapId: "cfl_gf",
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
        mapId: "cfl_gf",
        name: "Inspect Zone A",
        owner: "CNDE",
        status: "Draft",
        createdAt: "2025-11-17",
        notes: "Routine inspection",
      },
      {
        id: "m2",
        mapId: "cfl_gf",
        name: "Delivery Route 3",
        owner: "ANSCER ADMIN",
        status: "Scheduled",
        createdAt: "2025-11-16",
        notes: "Delivery to docks",
      },
      {
        id: "m3",
        mapId: "cfl_gf",
        name: "Battery Check",
        owner: "CNDE",
        status: "Completed",
        createdAt: "2025-11-15",
        notes: "Post-run check",
      },
    ];
    const [missions, setMissions] = useState(initialMissions);
    const [selectedMissionId, setSelectedMissionId] = useState(null);
    const handleSelectMission = (id) => setSelectedMissionId(id);
    const [missionFormOpen, setMissionFormOpen] = useState(false);
    const [missionForm, setMissionForm] = useState({
      name: "",
      owner: "",
      status: "Draft",
      notes: "",
    });

    // Add filtered getters for waypoints, zones, missions based on active map
    const getFilteredWaypoints = () => {
      if (!selectedMap) return [];
      const filtered = waypoints.filter((wp) => wp.mapId === selectedMap.id);
      console.log(`📍 getFilteredWaypoints for map ${selectedMap.id}:`, filtered);
      return filtered;
    };

    const getFilteredZones = () => {
      if (!selectedMap) return [];
      const filtered = zones.filter((z) => z.mapId === selectedMap.id);
      console.log(`📍 getFilteredZones for map ${selectedMap.id}:`, filtered);
      return filtered;
    };

    const getFilteredMissions = () => {
      if (!selectedMap) return [];
      const filtered = missions.filter((m) => m.mapId === selectedMap.id);
      console.log(`📍 getFilteredMissions for map ${selectedMap.id}:`, filtered);
      return filtered;
    };

    // users data + selection (fixes eslint no-undef)
    const [users, setUsers] = useState([
      {
        id: 1,
        username: "john_doe",
        email: "john.doe@example.com",
        company: "ANSCER Robotics",
        amr_type: "Type A",
        role: "user",
        approval: "Approved",
      },
      {
        id: 2,
        username: "jane_smith",
        email: "jane.smith@example.com",
        company: "CNDE IITM",
        amr_type: "Type B",
        role: "user",
        approval: "Pending",
      },
      {
        id: 3,
        username: "bob_wilson",
        email: "bob.wilson@example.com",
        company: "TechCorp",
        amr_type: "Type A",
        role: "user",
        approval: "Approved",
      },
      {
        id: 4,
        username: "alice_johnson",
        email: "alice.johnson@example.com",
        company: "Innovation Labs",
        amr_type: "Type C",
        role: "user",
        approval: "Rejected",
      },
      {
        id: 5,
        username: "charlie_brown",
        email: "charlie.brown@example.com",
        company: "RoboTech Inc",
        amr_type: "Type A",
        role: "user",
        approval: "Approved",
      },
      {
        id: 6,
        username: "diana_prince",
        email: "diana.prince@example.com",
        company: "Wonder Systems",
        amr_type: "Type B",
        role: "user",
        approval: "Approved",
      },
      {
        id: 7,
        username: "edward_norton",
        email: "edward.norton@example.com",
        company: "Norton Industries",
        amr_type: "Type C",
        role: "user",
        approval: "Pending",
      },
      {
        id: 8,
        username: "admin_user",
        email: "admin@example.com",
        company: "ANSCER Admin",
        amr_type: "Admin Console",
        role: "admin",
        approval: "Approved",
      },
    ]);


    const handleLogout = useCallback(() => {
      // clear tokens and go back to login
      clearAuthTokens();
      navigate("/");
    }, [navigate]);

    const initialZones = [
      {
        id: "z1",
        mapId: "cfl_gf",
        name: "Assembly Lane",
        category: "Safe",
        active: true,
        geometry: "Polygon(32,18…)",
        createdAt: "2025-11-17",
      },
      {
        id: "z2",
        mapId: "cfl_gf",
        name: "Battery Bay",
        category: "No-Go",
        active: true,
        geometry: "Polygon(27,11…)",
        createdAt: "2025-11-15",
      },
      {
        id: "z3",
        mapId: "cfl_gf",
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

    // apply a simple theme state for whole app (light|dark|system)
    const [selectedTheme, setSelectedTheme] = useState("light");

    useEffect(() => {
      try {
        document.documentElement.setAttribute("data-theme", selectedTheme);
      } catch { }
    }, [selectedTheme]);



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
          // Select the active map (not the "zones" dummy entry)
          const activeMap = mapsList.find((m) => (m.status || "").toLowerCase() === "active");
          if (activeMap) setSelectedMap(activeMap);
        } else if (id === "waypoints") {
          // Select the active map so waypoints page shows that map's waypoints
          const activeMap = mapsList.find((m) => (m.status || "").toLowerCase() === "active");
          if (activeMap) setSelectedMap(activeMap);
        } else if (id === "missions") {
          // Select the active map so missions page shows that map's missions
          const activeMap = mapsList.find((m) => (m.status || "").toLowerCase() === "active");
          if (activeMap) setSelectedMap(activeMap);
        } else if (id === "users") {
          // Users page doesn't need a selectedMap — it shows all users
          // Don't change selectedMap so it stays on the current map
          // RightPane will handle the users display directly without map filtering
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

    // Handle image file selection for map modal — read as data URL and store in mapForm.image
    const handleMapImageChange = (e) => {
      const file = e.target.files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        setMapForm((prev) => ({ ...prev, image: reader.result || "" }));
      };
      reader.readAsDataURL(file);
    };

    const triggerMapImagePicker = () => {
      try {
        if (mapImageInputRef.current) mapImageInputRef.current.click();
      } catch (err) {
        console.warn("Image picker failed", err);
      }
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
        const mapId = map.id;

        console.log(`🗺️ Activating map: ${mapId}`);

        // Optimistically update UI: mark chosen map Active, clear previous active status
        setMapsList((prev) =>
          prev.map((m) => {
            if (m.id === map.id) return { ...m, status: "Active" };
            if ((m.status || "").toLowerCase() === "active" && m.id !== map.id) return { ...m, status: "" };
            return m;
          }),
        );

        // Set selected map
        setSelectedMap({ ...map, status: "Active" });

        // Check if this map already has waypoints/zones/missions
        const hasWaypoints = waypoints.some(wp => wp.mapId === mapId);
        const hasZones = zones.some(z => z.mapId === mapId);
        const hasMissions = missions.some(m => m.mapId === mapId);

        console.log(`📍 Existing data - Waypoints: ${hasWaypoints}, Zones: ${hasZones}, Missions: ${hasMissions}`);

        // Create and set waypoints immediately
        if (!hasWaypoints) {
          const defaultWaypoints = [
            {
              id: `wp-${mapId}-delivery`,
              mapId: mapId,
              name: "Delivery",
              category: "Normal",
              geom: "Point(10 10)",
              notes: `Delivery waypoint for ${map.name}`,
              active: true,
              createdAt: new Date().toLocaleString(),
            },
            {
              id: `wp-${mapId}-shelf1`,
              mapId: mapId,
              name: "Shelf_1",
              category: "Normal",
              geom: "Point(50 30)",
              notes: `Shelf 1 waypoint for ${map.name}`,
              active: true,
              createdAt: new Date().toLocaleString(),
            },
            {
              id: `wp-${mapId}-shelf2`,
              mapId: mapId,
              name: "Shelf_2",
              category: "Normal",
              geom: "Point(80 50)",
              notes: `Shelf 2 waypoint for ${map.name}`,
              active: true,
              createdAt: new Date().toLocaleString(),
            },
            {
              id: `wp-${mapId}-shelf3`,
              mapId: mapId,
              name: "Shelf_3",
              category: "Normal",
              geom: "Point(20 70)",
              notes: `Shelf 3 waypoint for ${map.name}`,
              active: true,
              createdAt: new Date().toLocaleString(),
            },
            {
              id: `wp-${mapId}-shelf4`,
              mapId: mapId,
              name: "Shelf_4",
              category: "Normal",
              geom: "Point(60 80)",
              notes: `Shelf 4 waypoint for ${map.name}`,
              active: true,
              createdAt: new Date().toLocaleString(),
            },
          ];
          console.log(`✅ Creating ${defaultWaypoints.length} waypoints for map ${mapId}`);
          setWaypoints(prev => [...prev, ...defaultWaypoints]);
        }

        // Create and set zones immediately
        if (!hasZones) {
          const defaultZones = [
            {
              id: `zone-${mapId}-1`,
              mapId: mapId,
              name: `${map.name} - Safe Zone`,
              category: "Safe",
              geometry: "Polygon((0,0),(100,0),(100,50),(0,50))",
              active: true,
              createdAt: new Date().toLocaleString(),
            },
            {
              id: `zone-${mapId}-2`,
              mapId: mapId,
              name: `${map.name} - Caution Area`,
              category: "Caution",
              geometry: "Polygon((0,50),(50,50),(50,100),(0,100))",
              active: true,
              createdAt: new Date().toLocaleString(),
            },
            {
              id: `zone-${mapId}-3`,
              mapId: mapId,
              name: `${map.name} - Restricted`,
              category: "No-Go",
              geometry: "Polygon((50,50),(100,50),(100,100),(50,100))",
              active: true,
              createdAt: new Date().toLocaleString(),
            },
          ];
          console.log(`✅ Creating ${defaultZones.length} zones for map ${mapId}`);
          setZones(prev => [...prev, ...defaultZones]);
        }

        // Create and set missions immediately
        if (!hasMissions) {
          const defaultMissions = [
            {
              id: `mission-${mapId}-delivery`,
              mapId: mapId,
              name: "Delivery",
              owner: "CNDE",
              status: "Draft",
              notes: `Delivery mission for ${map.name}`,
              createdAt: new Date().toLocaleString(),
              iteration: 1,
            },
            {
              id: `mission-${mapId}-shelf1`,
              mapId: mapId,
              name: "Shelf_1",
              owner: "CNDE",
              status: "Draft",
              notes: `Shelf 1 mission for ${map.name}`,
              createdAt: new Date().toLocaleString(),
              iteration: 1,
            },
            {
              id: `mission-${mapId}-shelf2`,
              mapId: mapId,
              name: "Shelf_2",
              owner: "CNDE",
              status: "Draft",
              notes: `Shelf 2 mission for ${map.name}`,
              createdAt: new Date().toLocaleString(),
              iteration: 1,
            },
            {
              id: `mission-${mapId}-shelf3`,
              mapId: mapId,
              name: "Shelf_3",
              owner: "CNDE",
              status: "Draft",
              notes: `Shelf 3 mission for ${map.name}`,
              createdAt: new Date().toLocaleString(),
              iteration: 1,
            },
            {
              id: `mission-${mapId}-shelf4`,
              mapId: mapId,
              name: "Shelf_4",
              owner: "CNDE",
              status: "Draft",
              notes: `Shelf 4 mission for ${map.name}`,
              createdAt: new Date().toLocaleString(),
              iteration: 1,
            },
          ];
          console.log(`✅ Creating ${defaultMissions.length} missions for map ${mapId}`);
          setMissions(prev => [...prev, ...defaultMissions]);
        }

        // Show success toasts
        if (!hasWaypoints) {
          toast.success(`Created 5 waypoints for ${map.name}`);
        }
        if (!hasZones) {
          toast.success(`Created 3 zones for ${map.name}`);
        }
        if (!hasMissions) {
          toast.success(`Created 5 missions for ${map.name}`);
        }
        toast.success(`Activated map: ${map.name}`);

        // Persist change to server (best-effort)
        try {
          await requestV1(`/maps/${map.id}`, {
            method: "PATCH",
            body: JSON.stringify({ status: "Active" }),
          });
        } catch (err) {
          console.warn("Failed to persist map activation:", err);
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
          if (error.message === "No access token available") {
            clearAuthTokens();
            navigate("/");
          }
          throw error;
        }
      },
      [navigate],
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
          return data.reply ? [...updated, data.reply] : updated;
        });
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
            // Normalize active status: ensure exactly one Active map locally
            const items = mapsRes.items.slice();
            const firstActiveIdx = items.findIndex(
              (m) => (m.status || "").toLowerCase() === "active",
            );
            let normalized;
            if (firstActiveIdx === -1) {
              // no active => mark first as Active
              normalized = items.map((m, idx) => (idx === 0 ? { ...m, status: "Active" } : { ...m, status: m.status || "" }));
            } else {
              // multiple active possible — keep firstActiveIdx active and clear others
              normalized = items.map((m, idx) =>
                idx === firstActiveIdx ? { ...m, status: "Active" } : { ...m, status: (m.status && m.status.toLowerCase() === "active") ? "" : (m.status || "") },
              );
            }
            setMapsList(normalized);
            // pick the active map as selected
            const activeMap = normalized.find((m) => (m.status || "").toLowerCase() === "active") || normalized[0];
            setSelectedMap(activeMap || null);
          }
          if (Array.isArray(zonesRes.items)) {
            setZones(zonesRes.items);
          }
          if (Array.isArray(waypointsRes.items)) {
            // Ensure all waypoints have mapId from server
            const waypointsWithMapId = waypointsRes.items.map(wp => ({
              ...wp,
              mapId: wp.mapId || "cfl_gf", // default to first map if missing
            }));
            setWaypoints(waypointsWithMapId);
          }
          if (Array.isArray(missionsRes.items)) {
            // Ensure all missions have mapId from server
            const missionsWithMapId = missionsRes.items.map(m => ({
              ...m,
              mapId: m.mapId || "cfl_gf", // default to first map if missing
            }));
            setMissions(missionsWithMapId);
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

    return (
      <div
        ref={layoutRef}
        className={`main-container ${rightPage ? "has-right-pane" : ""}`}
      >
        {/* Header component (emergency, battery, lock, fullscreen, logout, dropdown) */}
        <Header
          handleLogout={handleLogout}
          batteryStatus={batteryStatus}
          layoutRef={layoutRef}
          onLockChange={(locked) => setIsLocked(!!locked)}
          bridgeStatus={bridgeStatus}
          startBridgeConnection={startBridgeConnection}
        />

        <div className="content-wrap">
          <Sidebar
            onSelect={handleSidebarSelect}
            onBack={() => {
              if (rightPage) setRightPage(null);
            }}
          />

          {/* Right pane moved to separate component */}
          {rightPage && (
            <RightPane
              rightPage={rightPage}
              setRightPage={setRightPage}
              // maps
              mapsList={mapsList}
              setMapsList={setMapsList}
              selectedMap={selectedMap}
              setSelectedMap={setSelectedMap}
              createNewMapImmediate={createNewMapImmediate}
              handleActivateMap={handleActivateMap}
              handleMapAction={handleMapAction}
              mapSearchField={mapSearchField}
              setMapSearchField={setMapSearchField}
              mapSearchTerm={mapSearchTerm}
              setMapSearchTerm={setMapSearchTerm}
              requestV1={requestV1}
              toast={toast}
              // zones
              zones={getFilteredZones()}
              zoneFormOpen={zoneFormOpen}
              setZoneFormOpen={setZoneFormOpen}
              zoneForm={zoneForm}
              setZoneForm={setZoneForm}
              setZones={setZones}
              zoneSearchField={zoneSearchField}
              setZoneSearchField={setZoneSearchField}
              zoneSearchTerm={zoneSearchTerm}
              setZoneSearchTerm={setZoneSearchTerm}
              // waypoints
              waypoints={getFilteredWaypoints()}
              setWaypoints={setWaypoints}
              waypointFormOpen={waypointFormOpen}
              setWaypointFormOpen={setWaypointFormOpen}
              waypointForm={waypointForm}
              setWaypointForm={setWaypointForm}
              handleSelectWaypoint={handleSelectWaypoint}
              setSelectedWaypointId={setSelectedWaypointId}
              // users
              users={users}
              setUsers={setUsers}
              usersLoading={usersLoading}
              usersError={usersError}
              loadUsers={loadUsers}
              selectedUserId={selectedUserId}
              setSelectedUserId={setSelectedUserId}
              userActionLoading={userActionLoading}
              handleResetUserPassword={handleResetUserPassword}
              // missions
              missions={getFilteredMissions()}
              missionFormOpen={missionFormOpen}
              setMissionFormOpen={setMissionFormOpen}
              missionForm={missionForm}
              setMissionForm={setMissionForm}
              selectedMissionId={selectedMissionId}
              setSelectedMissionId={setSelectedMissionId}
              handleSelectMission={handleSelectMission}
              setMissions={setMissions}
              // analytics / diagnostics / logs / mission history / bags
              analyticsSummary={analyticsSummary}
              analyticsSeries={analyticsSeries}
              analyticsAlerts={analyticsAlerts}
              diagnosticsPanels={diagnosticsPanels}
              logEvents={logEvents}
              missionHistory={missionHistory}
              bagFiles={bagFiles}
              // settings / account / appearance / security / integrations
              robotSettingsState={robotSettingsState}
              toggleRobotSetting={toggleRobotSetting}
              accountProfile={accountProfile}
              handleAccountChange={handleAccountChange}
              profileError={profileError}
              profileSuccess={profileSuccess}
              profileSaving={profileSaving}
              handleSaveProfile={handleSaveProfile}
              selectedTheme={selectedTheme}
              setSelectedTheme={setSelectedTheme}
              securityPreferences={securityPreferences}
              toggleSecurityPref={toggleSecurityPref}
              securityEvents={securityEvents}
              integrationItems={integrationItems}
              toggleIntegrationStatus={toggleIntegrationStatus}
              // stats
              overview={overview}
              missionTrend={missionTrend}
              monthlyMovement={monthlyMovement}
              batterySeries={batterySeries}
              batteryStatus={batteryStatus}
              turns={turns}
              statsLoading={statsLoading}
              statsError={statsError}
              lineChartSize={lineChartSize}
              buildLinePath={buildLinePath}
              buildSimplePath={buildSimplePath}
              analyticsChartSize={analyticsChartSize}
              analyticsPath={analyticsPath}
              // chat
              chatMessages={chatMessages}
              chatInput={chatInput}
              setChatInput={setChatInput}
              handleSendMessage={handleSendMessage}
              handleKeyPress={handleKeyPress}
              isRecording={isRecording}
              handleMicClick={handleMicClick}
              isTyping={isTyping}
              handleSuggestionClick={handleSuggestionClick}
              chatQuickPrompts={chatQuickPrompts}
              chatContainerRef={chatContainerRef}
              formatTimestamp={formatTimestamp}
              latestMessage={latestMessage}
            />
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
                {/* render selected map image if available, otherwise empty white area */}
                {selectedMap?.image ? (
                  <img
                    src={selectedMap.image}
                    alt={selectedMap.name || "Map preview"}
                    style={{ width: "100%", height: "100%", objectFit: "contain", display: "block" }}
                  />
                ) : (

                  <div style={{ width: "100%", height: "100%" }} aria-hidden="true" />
                )}
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
                <div className={`joystick-overlay ${minimizedMain ? "minimized" : ""}`}></div>
                <JoyStick width={140} height={140} onMove={handleJoystickMove} />
              </div>
            </div>
        </div>
      </main>
      </div >

  );
};

export default MainPage;
