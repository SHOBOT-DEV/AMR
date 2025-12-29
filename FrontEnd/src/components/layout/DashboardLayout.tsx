import React, {
  useRef,
  useEffect,
  useState,
  useCallback,
  type KeyboardEvent,
} from "react";
import { useNavigate } from "react-router-dom";
import { toast, Toaster } from "react-hot-toast";

import Sidebar from "./SideBar.tsx";
import Header from "./Header.tsx";
import MapArea from "../map/MapArea.tsx";
import RightPane from "../test/RightPane.tsx";
import JoyStick from "../map/JoyStick.tsx";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../../utils/auth";

type OverviewStats = {
  totalKm: number;
  missionsCompleted: number;
  avgSpeed: number;
  operatingHours: number;
};

type MissionTrendEntry = {
  label: string;
  completed: number;
  incidents: number;
};

type MonthlyMovementEntry = {
  month: string;
  km: number;
};

type BatteryPoint = {
  time: string;
  voltage: number;
  power: number;
};

type BatteryStatus = {
  packVoltage: number;
  packCurrent: number;
  stateOfCharge: number;
  temperature: string;
  cycles: number;
  health: string;
  cells?: Array<{ id: string; voltage: number }>;
};

type StatsData = {
  overview: OverviewStats & { deltaKm?: number; missionSuccessRate?: number };
  batterySeries: BatteryPoint[];
  batteryStatus: BatteryStatus;
  missionTrend: MissionTrendEntry[];
  monthlyMovement: MonthlyMovementEntry[];
  turns: { left: number; right: number };
};

type JoystickState = {
  x: number;
  y: number;
  force: number;
  angle: number;
  type: string | null;
};

type BridgeStatus = {
  connected: boolean;
  endpoint: string;
  error?: string;
};

const API_V1_BASE = `${API_BASE}/api/v1`;
const FALLBACK_STATS: StatsData = {
  overview: {
    totalKm: 182.4,
    missionsCompleted: 47,
    avgSpeed: 1.8,
    operatingHours: 326,
    deltaKm: 4.3,
    missionSuccessRate: 98,
  },
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
  turns: { left: 132, right: 148 },
};

const SAMPLE_MAPS = [
  {
    id: "cfl_gf",
    name: "CFL_GF",
    createdBy: "CNDE IITM",
    image: "",
    status: "Active",
    category: "Production",
    createdAt: "2025-11-17",
  },
  {
    id: "arena",
    name: "Shobot Arena",
    createdBy: "ANSCER ADMIN",
    image: "/images/maps/shobot_arena.png",
    status: "Draft",
    category: "Testing",
    createdAt: "2025-11-16",
  },
];

const SAMPLE_WAYPOINTS = [
  {
    id: "wp1",
    mapId: "cfl_gf",
    name: "WP_A",
    category: "Nav",
    active: true,
    geom: "Point(40 12)",
    createdAt: "2025-11-17",
    notes: "Primary pickup",
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
];

const SAMPLE_ZONES = [
  {
    id: "z1",
    mapId: "cfl_gf",
    name: "Dock Corridor",
    category: "Safe",
    active: true,
    geometry: "Polygon(88,44…)",
    createdAt: "2025-11-16",
  },
  {
    id: "z2",
    mapId: "cfl_gf",
    name: "Battery Corner",
    category: "No-Go",
    active: true,
    geometry: "Polygon(27,11…)",
    createdAt: "2025-11-15",
  },
];

const SAMPLE_MISSIONS = [
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
];

const SAMPLE_USERS = [
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
    username: "alice_johnson",
    email: "alice.johnson@example.com",
    company: "Innovation Labs",
    amr_type: "Type C",
    role: "user",
    approval: "Rejected",
  },
];

const SAMPLE_ANALYTICS_SUMMARY = [
  { label: "Incidents", value: 2, trend: "+1 vs last week" },
  { label: "Stops Issued", value: 14, trend: "-3 vs last week" },
  { label: "Battery Swaps", value: 5, trend: "Stable" },
];

const SAMPLE_ANALYTICS_SERIES = [12, 18, 22, 16, 24, 26, 20];

const SAMPLE_ANALYTICS_ALERTS = [
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
];

const SAMPLE_DIAGNOSTICS = [
  { id: "battery", title: "Battery Health", value: "93%", status: "Nominal", detail: "Cells balanced" },
  { id: "motors", title: "Drive Motors", value: "Temp 48°C", status: "Monitoring", detail: "Torque variance +3%" },
];

const SAMPLE_LOG_EVENTS = [
  { id: "log1", ts: "10:42:01", system: "Navigation", message: "Replanned path around blocked aisle", level: "info" },
  { id: "log2", ts: "10:15:22", system: "Safety", message: "Emergency stop acknowledged", level: "warn" },
];

const SAMPLE_MISSION_HISTORY = [
  { id: "mh1", mission: "Inspect Zone A", window: "08:00–08:18", outcome: "Completed", notes: "No issues" },
  { id: "mh2", mission: "Delivery Route 3", window: "08:30–09:10", outcome: "Delayed", notes: "Obstacle at Dock Tunnel" },
];

const SAMPLE_BAG_FILES = [
  { id: "bag1", name: "mission-0915.bag", duration: "15m", size: "1.4 GB", status: "Uploaded" },
  { id: "bag2", name: "mission-1030.bag", duration: "26m", size: "2.7 GB", status: "Processing" },
];

const SAMPLE_SECURITY_EVENTS = [
  { id: "sec1", ts: "09:44", actor: "ops-admin", action: "API token created", context: "Main console" },
  { id: "sec2", ts: "08:12", actor: "robot-01", action: "Cert renewed", context: "Device" },
];

const SAMPLE_INTEGRATIONS = [
  { id: "rest", name: "REST API", status: "Connected", description: "Push missions from MES" },
  { id: "slack", name: "Slack Bot", status: "Disconnected", description: "Alerts to #robot-ops" },
  { id: "grafana", name: "Grafana", status: "Connected", description: "Telemetry dashboards" },
];

const SAMPLE_CHAT_MESSAGES = [
  {
    id: 1,
    text: "Hello! I'm your robot assistant. How can I help you today?",
    sender: "robot",
    timestamp: new Date().toISOString(),
    status: "Delivered",
  },
];

const CHAT_QUICK_PROMPTS = [
  "Provide current mission status",
  "Return to docking station",
  "Begin perimeter scan",
  "Share latest sensor alerts",
];

const LINE_CHART_SIZE = { width: 420, height: 180 };
const ANALYTICS_CHART_SIZE = { width: 280, height: 80 };

const Dashboard: React.FC = () => {
  const navigate = useNavigate();
  const layoutRef = useRef<HTMLDivElement | null>(null);

  const [isLocked, setIsLocked] = useState(false);
  const [emergencyClicked, setEmergencyClicked] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [rightPage, setRightPage] = useState<string | null>(null);
  const [minimizedMain, setMinimizedMain] = useState(false);
  
  // Auto-minimize map when right pane opens
  useEffect(() => {
    setMinimizedMain(Boolean(rightPage));
  }, [rightPage]);

  const [zoomLevel, setZoomLevel] = useState<number>(1);

  const handleZoom = (delta: number): void => {
    setZoomLevel(prev => Math.min(Math.max(prev + delta, 0.1), 3));
  };

  const toggleMapZoom = (): void => {
    setZoomLevel(prev => prev === 1 ? 1.8 : 1);
  };

  const [joystickState, setJoystickState] = useState<JoystickState>({
    x: 0,
    y: 0,
    force: 0,
    angle: 0,
    type: null,
  });

  const handleJoystickMove = useCallback((data: Partial<JoystickState>) => {
    const newState: JoystickState = {
      x: data.x ?? 0,
      y: data.y ?? 0,
      force: data.force ?? 0,
      angle: data.angle ?? 0,
      type: data.type || "joystick",
    };
    setJoystickState(newState);
  }, []);

  const [mapsList, setMapsList] = useState<any[]>(SAMPLE_MAPS);
  const [selectedMap, setSelectedMap] = useState<any>(SAMPLE_MAPS[0]);
  const [mapSearchField, setMapSearchField] = useState<string>("any");
  const [mapSearchTerm, setMapSearchTerm] = useState<string>("");

  const [zones, setZones] = useState<any[]>(SAMPLE_ZONES);
  const [zoneFormOpen, setZoneFormOpen] = useState(false);
  const [zoneForm, setZoneForm] = useState({
    name: "",
    category: "Safe",
    geometry: "",
    active: true,
  });
  const [zoneSearchField, setZoneSearchField] = useState("any");
  const [zoneSearchTerm, setZoneSearchTerm] = useState("");

  const [waypoints, setWaypoints] = useState<any[]>(SAMPLE_WAYPOINTS);
  const [waypointFormOpen, setWaypointFormOpen] = useState(false);
  const [waypointForm, setWaypointForm] = useState({
    name: "",
    category: "Nav",
    geom: "",
    notes: "",
    active: true,
  });
  const [selectedWaypointId, setSelectedWaypointId] = useState<string | null>(null);
  const handleSelectWaypoint = useCallback((waypointId: string | null) => {
    setSelectedWaypointId(waypointId);
  }, []);

  const [missions, setMissions] = useState<any[]>(SAMPLE_MISSIONS);
  const [selectedMissionId, setSelectedMissionId] = useState<string | null>(null);
  const handleSelectMission = useCallback((missionId: string) => {
    setSelectedMissionId(missionId);
  }, []);
  const [missionFormOpen, setMissionFormOpen] = useState(false);
  const [missionForm, setMissionForm] = useState({
    name: "",
    owner: "",
    status: "Draft",
    notes: "",
  });

  const [users, setUsers] = useState<any[]>(SAMPLE_USERS);
  const [usersLoading, setUsersLoading] = useState(false);
  const [usersError, setUsersError] = useState("");
  const [selectedUserId, setSelectedUserId] = useState<number | null>(null);
  const [userActionLoading, setUserActionLoading] = useState(false);

  const loadUsers = useCallback(() => {
    setUsersLoading(true);
    setTimeout(() => setUsersLoading(false), 500);
  }, []);

  const handleResetUserPassword = useCallback((userId: number) => {
    setUserActionLoading(true);
    setTimeout(() => {
      setUserActionLoading(false);
      toast.success(`Reset link sent for user ${userId}`);
    }, 700);
  }, []);

  const [analyticsSummary] = useState(SAMPLE_ANALYTICS_SUMMARY);
  const [analyticsSeries] = useState<number[]>(SAMPLE_ANALYTICS_SERIES);
  const [analyticsAlerts] = useState(SAMPLE_ANALYTICS_ALERTS);
  const [diagnosticsPanels] = useState(SAMPLE_DIAGNOSTICS);
  const [logEvents] = useState(SAMPLE_LOG_EVENTS);
  const [missionHistory] = useState(SAMPLE_MISSION_HISTORY);
  const [bagFiles] = useState(SAMPLE_BAG_FILES);

  const [robotSettingsState, setRobotSettingsState] = useState<Record<string, boolean>>({
    autopilot: true,
    safeMode: true,
    remoteDiagnostics: false,
    pathOptimization: true,
  });
  const toggleRobotSetting = useCallback((key: string) => {
    setRobotSettingsState((prev) => ({ ...prev, [key]: !prev[key] }));
  }, []);

  const [accountProfile, setAccountProfile] = useState({
    fullName: "Ops Admin",
    email: "ops@example.com",
    team: "Operations",
    shift: "Day",
  });
  const [profileSaving, setProfileSaving] = useState(false);
  const [profileError, setProfileError] = useState("");
  const [profileSuccess, setProfileSuccess] = useState("");

  const handleAccountChange = useCallback((field: string, value: string) => {
    setAccountProfile((prev) => ({ ...prev, [field]: value }));
  }, []);

  const handleSaveProfile = useCallback(() => {
    setProfileSaving(true);
    setProfileError("");
    setProfileSuccess("");
    setTimeout(() => {
      setProfileSaving(false);
      setProfileSuccess("Profile updated");
    }, 600);
  }, []);

  const [selectedTheme, setSelectedTheme] = useState("light");

  const [securityPreferences, setSecurityPreferences] = useState({
    twoFactor: true,
    autoLock: true,
    anomalyAlerts: true,
  });
  const [securityEvents] = useState(SAMPLE_SECURITY_EVENTS);
  const toggleSecurityPref = useCallback((key: string) => {
    setSecurityPreferences((prev) => ({ ...prev, [key]: !prev[key] }));
  }, []);

  const [integrationItems, setIntegrationItems] = useState(SAMPLE_INTEGRATIONS);
  const toggleIntegrationStatus = useCallback((id: string) => {
    setIntegrationItems((items) =>
      items.map((item) =>
        item.id === id
          ? { ...item, status: item.status === "Connected" ? "Disconnected" : "Connected" }
          : item,
      ),
    );
  }, []);

  const [statsData, setStatsData] = useState<StatsData>(FALLBACK_STATS);
  const [statsError, setStatsError] = useState("");
  const [statsLoading, setStatsLoading] = useState(false);

  const lineChartSize = LINE_CHART_SIZE;
  const analyticsChartSize = ANALYTICS_CHART_SIZE;

  const buildLinePath = useCallback(
    (series: BatteryPoint[], key: "voltage" | "power") => {
      if (!series.length) return "";
      const values = series.map((point) => point[key]);
      const max = Math.max(...values);
      const min = Math.min(...values);
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
    },
    [lineChartSize.height, lineChartSize.width],
  );

  const buildSimplePath = useCallback(
    (points: number[], size = analyticsChartSize) => {
      if (!points.length) return "";
      const max = Math.max(...points);
      const min = Math.min(...points);
      const range = max - min || 1;
      const stepX = points.length > 1 ? size.width / (points.length - 1) : size.width;
      return points
        .map((value, index) => {
          const x = index * stepX;
          const normalized = (value - min) / range;
          const y = size.height - normalized * size.height;
          return `${x},${y}`;
        })
        .join(" ");
    },
    [analyticsChartSize],
  );

  const analyticsPath = buildSimplePath(analyticsSeries, analyticsChartSize);
  const [bridgeStatus, setBridgeStatus] = useState<BridgeStatus>({
    connected: false,
    endpoint: "http://localhost:8000",
    error: "",
  });

  const requestV1 = useCallback(
    async (path: string, options: RequestInit = {}) => {
      const headers: HeadersInit = {
        Accept: "application/json",
        ...(options.headers || {}),
      };
      if (options.body && !(headers as Record<string, string>)["Content-Type"]) {
        (headers as Record<string, string>)["Content-Type"] = "application/json";
      }

      const response = await fetchWithAuth(`${API_V1_BASE}${path}`, {
        ...options,
        headers,
      });
      const data = await response.json().catch(() => ({}));
      if (response.status === 401) {
        clearAuthTokens();
        navigate("/");
        throw new Error("Unauthorized");
      }
      if (!response.ok) throw new Error((data as any).message || "Request failed");
      return data;
    },
    [navigate],
  );

  const handleLogout = useCallback(() => {
    clearAuthTokens();
    navigate("/");
  }, [navigate]);

  const LOCK_TOAST_ID = "lock-toast";

  const handleToggleLock = useCallback(() => {
    setIsLocked((prev) => {
      const next = !prev;
      toast.dismiss(LOCK_TOAST_ID);
      if (next) {
        toast.error("Console locked", { id: LOCK_TOAST_ID });
      } else {
        toast.success("Console unlocked", { id: LOCK_TOAST_ID });
      }
      return next;
    });
  }, []);

  const createNewMapImmediate = useCallback(() => {
    setMapsList((prev) => {
      const newMap = {
        id: `map-${Date.now()}`,
        name: `New Map ${prev.length + 1}`,
        createdBy: "Operator",
        image: "",
        status: "Draft",
        category: "General",
        createdAt: new Date().toISOString().slice(0, 10),
      };
      toast.success("Map placeholder created");
      return [newMap, ...prev];
    });
  }, []);

  const handleActivateMap = useCallback((map: any) => {
    setSelectedMap(map);
  }, []);

  const handleMapAction = useCallback(
    (action: string, map: any) => {
      if (action === "delete") {
        setMapsList((prev) => prev.filter((entry) => entry.id !== map.id));
        setSelectedMap((prev) => (prev?.id === map.id ? null : prev));
        toast.success("Map deleted");
        return;
      }
      toast("Action not implemented");
    },
    [],
  );

  const [chatMessages, setChatMessages] = useState(SAMPLE_CHAT_MESSAGES);
  const [chatInput, setChatInput] = useState("");
  const [isTyping, setIsTyping] = useState(false);
  const chatQuickPrompts = CHAT_QUICK_PROMPTS;
  const chatContainerRef = useRef<HTMLDivElement | null>(null);
  const latestMessage = chatMessages[chatMessages.length - 1] ?? null;

  const formatTimestamp = useCallback((timestamp: string) => {
    try {
      return new Date(timestamp).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
    } catch {
      return "--:--";
    }
  }, []);

  const handleSuggestionClick = useCallback((prompt: string) => {
    setChatInput(prompt);
  }, []);

  const handleSendMessage = useCallback(() => {
    if (!chatInput.trim()) return;
    const outbound = {
      id: Date.now(),
      text: chatInput.trim(),
      sender: "human",
      timestamp: new Date().toISOString(),
      status: "Sent",
    };
    setChatMessages((prev) => [...prev, outbound]);
    setChatInput("");
    setIsTyping(true);
    setTimeout(() => {
      setChatMessages((prev) => [
        ...prev,
        {
          id: Date.now(),
          text: "Acknowledged. Executing placeholder response.",
          sender: "robot",
          timestamp: new Date().toISOString(),
          status: "Delivered",
        },
      ]);
      setIsTyping(false);
    }, 900);
  }, [chatInput]);

  const handleKeyPress = useCallback(
    (event: KeyboardEvent<HTMLTextAreaElement>) => {
      if (event.key === "Enter" && !event.shiftKey) {
        event.preventDefault();
        handleSendMessage();
      }
    },
    [handleSendMessage],
  );

  // Allow unlocking via Escape key
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (!isLocked) return;
      if (e.key === "Escape") {
        handleToggleLock();
      }
    };
    window.addEventListener("keydown", onKey as any);
    return () => window.removeEventListener("keydown", onKey as any);
  }, [isLocked, handleToggleLock]);

  return (
    <div
      ref={layoutRef}
      className={`flex h-screen w-screen flex-col overflow-hidden bg-slate-50 ${
        isLocked ? "pointer-events-none select-none filter blur-sm" : ""
      }`}
    >
      <Toaster position="top-center" />

      {/* Lock Overlay - shows when locked */}
      {isLocked && (
        <div
          className="fixed inset-0 z-[9998] flex cursor-not-allowed items-center justify-center bg-black/50 backdrop-blur-md"
          onClick={(e) => {
            e.stopPropagation();
            e.preventDefault();
          }}
        >
          <div className="rounded-full border border-white/20 bg-white/10 px-8 py-3 font-bold text-white backdrop-blur-xl">
            Console Locked — Press Esc to Unlock
          </div>
        </div>
      )}

      {/* Unlock Button - ONLY visible when locked (floating on top) */}
      {isLocked && (
        <button
          className="pointer-events-auto fixed right-6 top-6 z-[9999] rounded-full bg-white/10 p-3 text-white transition-all hover:bg-white/20"
          onClick={handleToggleLock}
          aria-label="Unlock console"
          title="Unlock Console"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
            <path d="M7 11V7a5 5 0 0 1 9.9-1"></path>
          </svg>
        </button>
      )}

      <Header
        bridgeStatus={bridgeStatus}
        emergencyClicked={emergencyClicked}
        setEmergencyClicked={setEmergencyClicked}
        handleLogout={handleLogout}
        isLocked={isLocked}
        setIsLocked={setIsLocked}
        batteryLevel={batteryLevel}
        layoutRef={layoutRef}
      />

      <div className="relative flex h-full flex-1 pt-14">

        <Sidebar
          onSelect={(id) => {
            const panelIds = new Set([
              "maps",
              "zones",
              "waypoints",
              "missions",
              "users",
              "analytics",
              "diagnostics",
              "logs",
              "camera",
              "bridge",
              "mission-logs",
              "robot-bags",
              "chat",
              "stats",
              "robot-settings",
              "account",
              "appearance",
              "security",
              "integrations",
            ]);
            setRightPage(panelIds.has(id) ? id : null);
          }}
          onBack={() => setRightPage(null)}
        />

        <div className="relative ml-16 flex flex-1 overflow-hidden">
          {/* Left content wrapper that shrinks when right pane opens */}
          <div
            className={`relative h-full ${
              minimizedMain
                ? "shrink-0 basis-1/2 flex-none"
                : "flex-auto basis-full"
            }`}
          >
            {/* Map Area */}
            <MapArea
              minimized={minimizedMain}
              zoomLevel={zoomLevel}
              selectedMap={selectedMap}
              toggleMapZoom={toggleMapZoom}
            />

            {/* Zoom Controls - Top Right within left content */}
            <div className="absolute right-6 top-20 z-20 flex flex-col gap-2 bg-white rounded-lg shadow-md border border-gray-200 overflow-hidden">
              <button
                onClick={() => handleZoom(0.1)}
                className="p-3 hover:bg-gray-50 active:bg-gray-100 text-slate-600 font-bold transition-colors"
                title="Zoom In"
                aria-label="Zoom in"
              >
                +
              </button>
              <div className="border-b border-gray-200"></div>
              <button
                onClick={() => handleZoom(-0.1)}
                className="p-3 hover:bg-gray-50 active:bg-gray-100 text-slate-600 font-bold transition-colors"
                title="Zoom Out"
                aria-label="Zoom out"
              >
                −
              </button>
            </div>

            {/* Joystick - bottom right within left content */}
            <div
              className={`
                absolute pointer-events-auto z-[25]
                ${
                  minimizedMain
                    ? "right-2 bottom-2 z-[30]"
                    : "right-8 bottom-8 max-[1000px]:right-4 max-[1000px]:bottom-4 max-[800px]:right-3 max-[800px]:bottom-3"
                }
              `}
            >
              <JoyStick width={140} height={140} onMove={handleJoystickMove} />
            </div>
          </div>

          {rightPage && (
            <RightPane
              // UI Control
              rightPage={rightPage}
              setRightPage={setRightPage}
              
              // Maps
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
              
              // Zones
              zones={zones}
              setZones={setZones}
              zoneFormOpen={zoneFormOpen}
              setZoneFormOpen={setZoneFormOpen}
              zoneForm={zoneForm}
              setZoneForm={setZoneForm}
              zoneSearchField={zoneSearchField}
              setZoneSearchField={setZoneSearchField}
              zoneSearchTerm={zoneSearchTerm}
              setZoneSearchTerm={setZoneSearchTerm}
              
              // Waypoints
              waypoints={waypoints}
              setWaypoints={setWaypoints}
              waypointFormOpen={waypointFormOpen}
              setWaypointFormOpen={setWaypointFormOpen}
              waypointForm={waypointForm}
              setWaypointForm={setWaypointForm}
              handleSelectWaypoint={handleSelectWaypoint}
              setSelectedWaypointId={setSelectedWaypointId}
              
              // Users
              users={users}
              usersLoading={usersLoading}
              usersError={usersError}
              loadUsers={loadUsers}
              selectedUserId={selectedUserId}
              setSelectedUserId={setSelectedUserId}
              userActionLoading={userActionLoading}
              handleResetUserPassword={handleResetUserPassword}
              
              // Missions
              missions={missions}
              setMissions={setMissions}
              missionFormOpen={missionFormOpen}
              setMissionFormOpen={setMissionFormOpen}
              missionForm={missionForm}
              setMissionForm={setMissionForm}
              selectedMissionId={selectedMissionId}
              setSelectedMissionId={setSelectedMissionId}
              handleSelectMission={handleSelectMission}
              
              // Data & Analytics
              analyticsSummary={analyticsSummary}
              analyticsSeries={analyticsSeries}
              analyticsAlerts={analyticsAlerts}
              diagnosticsPanels={diagnosticsPanels}
              logEvents={logEvents}
              missionHistory={missionHistory}
              bagFiles={bagFiles}
              analyticsChartSize={analyticsChartSize}
              analyticsPath={analyticsPath}
              
              // Settings & Account
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
              
              // Stats (spread specific stats fields if they are inside an object like statsData)
              overview={statsData.overview}
              missionTrend={statsData.missionTrend}
              monthlyMovement={statsData.monthlyMovement}
              batterySeries={statsData.batterySeries}
              batteryStatus={statsData.batteryStatus}
              turns={statsData.turns}
              statsLoading={statsLoading}
              statsError={statsError}
              lineChartSize={lineChartSize}
              buildLinePath={buildLinePath}
              buildSimplePath={buildSimplePath}
              
              // Chat
              chatMessages={chatMessages}
              chatInput={chatInput}
              setChatInput={setChatInput}
              handleSendMessage={handleSendMessage}
              handleKeyPress={handleKeyPress}
              isTyping={isTyping}
              handleSuggestionClick={handleSuggestionClick}
              chatQuickPrompts={chatQuickPrompts}
              chatContainerRef={chatContainerRef}
              formatTimestamp={formatTimestamp}
              latestMessage={latestMessage}
            />
          )}
        </div>
      </div>

      {isLocked ? (
        <div
          className="fixed inset-0 z-[9999] flex cursor-not-allowed items-center justify-center bg-black/50 backdrop-blur-md"
          onClick={() => toast.error("Screen is locked")}
        >
          <div className="text-center">
            <div className="rounded-full border border-white/20 bg-white/10 px-8 py-3 font-bold text-white backdrop-blur-xl">
              Console Locked — Press Esc to Unlock
            </div>
          </div>

          <button
            className="pointer-events-auto absolute right-6 top-6 rounded-full bg-white/10 p-3 text-white transition-all hover:bg-white/20"
            onClick={(event) => {
              event.stopPropagation();
              setIsLocked(false);
            }}
            aria-label="Unlock"
          />
        </div>
      ) : null}
    </div>
  );
};

export default Dashboard;
