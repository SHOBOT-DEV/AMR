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
import MapArea from "../map/MapArea";
import RightPane from "./RightPaneOpen.tsx";
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../../utils/auth";

type OverviewStats = {
  totalKm: number;
  missionsCompleted: number;
  avgSpeed: number;
  operatingHours: number;
};

type StatsData = {
  overview: OverviewStats;
  batterySeries: Array<Record<string, unknown>>;
  batteryStatus?: Record<string, unknown>;
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
  overview: { totalKm: 0, missionsCompleted: 0, avgSpeed: 0, operatingHours: 0 },
  batterySeries: [],
};

const Dashboard: React.FC = () => {
  const navigate = useNavigate();
  const layoutRef = useRef<HTMLDivElement | null>(null);

  const [isLocked, setIsLocked] = useState(false);
  const [emergencyClicked, setEmergencyClicked] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [rightPage, setRightPage] = useState<string | null>(null);
  const [minimizedMain, setMinimizedMain] = useState(false);
  useEffect(() => {
    setMinimizedMain(Boolean(rightPage));
  }, [rightPage]);

  const [joystickState, setJoystickState] = useState<JoystickState>({
    x: 0,
    y: 0,
    force: 0,
    angle: 0,
    type: null,
  });

  const [mapsList, setMapsList] = useState<any[]>([]);
  const [selectedMap, setSelectedMap] = useState<any>(null);
  const [zones, setZones] = useState<any[]>([]);
  const [statsData, setStatsData] = useState<StatsData>(FALLBACK_STATS);
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

  const handleJoystickMove = useCallback((data: Partial<JoystickState>) => {
    setJoystickState({
      x: data.x ?? 0,
      y: data.y ?? 0,
      force: data.force ?? 0,
      angle: data.angle ?? 0,
      type: data.type ?? "joystick",
    });
  }, []);

  return (
    <div
      ref={layoutRef}
      className={`flex h-screen w-screen flex-col overflow-hidden bg-slate-50 ${
        isLocked ? "pointer-events-none select-none filter blur-sm" : ""
      }`}
    >
      <Toaster position="top-center" />

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
          <MapArea
            minimized={minimizedMain}
            isLocked={isLocked}
            emergencyClicked={emergencyClicked}
            onJoystickMove={handleJoystickMove}
          />

          {rightPage ? (
            <RightPane
              page={rightPage}
              onClose={() => setRightPage(null)}
              mapsData={{
                list: mapsList,
                setList: setMapsList,
                selected: selectedMap,
                setSelected: setSelectedMap,
                requestV1,
              }}
              zonesData={{ list: zones, setList: setZones, requestV1 }}
              statsData={statsData}
            />
          ) : null}
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
