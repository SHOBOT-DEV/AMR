import React, { useRef, useEffect, useState, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import { toast, Toaster } from "react-hot-toast";

// Components
import Sidebar from "./SideBar";
import Header from "./Header.jsx";
import MapArea from "../map/MapArea";
import RightPane from "./RightPane";
// import BatteryModal from "../components/common/BatteryModal"; // Ensure this exists

// Utils
import { fetchWithAuth, clearAuthTokens, API_BASE } from "../../utils/auth";
// import { connectBridgeSocket, fetchBridgeStatus } from "../utils/amrBridge";

const API_V1_BASE = `${API_BASE}/api/v1`;
const FALLBACK_STATS = { 
  overview: { totalKm: 0, missionsCompleted: 0, avgSpeed: 0, operatingHours: 0 },
  batterySeries: [] 
};

const Dashboard = () => {
  const navigate = useNavigate();
  const layoutRef = useRef(null);
  
  // --- Global State ---
  const [isLocked, setIsLocked] = useState(false);
  const [emergencyClicked, setEmergencyClicked] = useState(false);
  // const [batteryModalOpen, setBatteryModalOpen] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [rightPage, setRightPage] = useState(null);
  
  // --- Data State ---
  const [mapsList, setMapsList] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [zones, setZones] = useState([]);
  const [statsData, setStatsData] = useState(FALLBACK_STATS);
  
  const [bridgeStatus, setBridgeStatus] = useState({
    connected: false,
    endpoint: "http://localhost:8000",
    error: ""
  });

  // --- Logic Helpers ---
  const requestV1 = useCallback(async (path, options = {}) => {
    const headers = { Accept: "application/json", ...(options.headers || {}) };
    if (options.body && !headers["Content-Type"]) headers["Content-Type"] = "application/json";
    
    const response = await fetchWithAuth(`${API_V1_BASE}${path}`, { ...options, headers });
    const data = await response.json().catch(() => ({}));
    if (response.status === 401) { clearAuthTokens(); navigate("/"); throw new Error("Unauthorized"); }
    if (!response.ok) throw new Error(data.message || "Request failed");
    return data;
  }, [navigate]);

  const handleLogout = useCallback(() => {
    clearAuthTokens();
    navigate("/");
  }, [navigate]);

  return (
    <div 
      ref={layoutRef} 
      className={`h-screen w-screen flex flex-col overflow-hidden bg-slate-50 ${isLocked ? "pointer-events-none select-none filter blur-sm" : ""}`}
    >
      <Toaster position="top-center" />
      
      {/* 1. Header (Fixed Top) */}
      <Header 
        bridgeStatus={bridgeStatus}
        emergencyClicked={emergencyClicked}
        setEmergencyClicked={setEmergencyClicked}
        handleLogout={handleLogout}
        isLocked={isLocked}
        setIsLocked={setIsLocked}
        batteryLevel={batteryLevel}
        // setBatteryModalOpen={setBatteryModalOpen}
        layoutRef={layoutRef}
      />

      {/* 2. Main Content Area */}
      {/* Added pt-14 to account for fixed header height */}
      <div className="flex flex-1 pt-14 h-full relative">
        
        {/* 3. Sidebar (Fixed Left) */}
        <Sidebar 
          onSelect={(id) => {
            const createIds = ["maps", "zones", "waypoints", "missions", "users"];
            const monitorIds = ["analytics", "diagnostics", "logs", "chat", "stats"];
            const settingsIds = ["robot-settings", "account", "appearance", "security", "integrations"];
            
            if ([...createIds, ...monitorIds, ...settingsIds].includes(id)) {
              setRightPage(id);
            } else {
              setRightPage(null);
            }
          }} 
          onBack={() => setRightPage(null)}
        />

        {/* 4. Map & Workspace (Flexible Center) */}
        {/* Added ml-16 to account for fixed sidebar width */}
        <div className="flex-1 ml-16 relative flex overflow-hidden">
          
          <MapArea 
            minimized={!!rightPage}
            isLocked={isLocked}
            emergencyClicked={emergencyClicked}
            onJoystickMove={(data) => console.log(data)}
          />

          {/* 5. Right Pane (Drawer) */}
          {/* Rendered conditionally inside flex container to push map or overlay */}
          {rightPage && (
            <RightPane 
              page={rightPage} 
              onClose={() => setRightPage(null)}
              mapsData={{ list: mapsList, setList: setMapsList, selected: selectedMap, setSelected: setSelectedMap, requestV1 }}
              zonesData={{ list: zones, setList: setZones, requestV1 }}
              statsData={statsData}
            />
          )}
        </div>
      </div>

      {/* Modals */}
      {/* {batteryModalOpen && (
        <BatteryModal 
          status={statsData.batteryStatus || {}} 
          onClose={() => setBatteryModalOpen(false)} 
        />
      )} */}

      {/* Lock Screen Overlay */}
      {isLocked && (
        <div 
          className="fixed inset-0 z-[9999] flex items-center justify-center bg-black/50 backdrop-blur-md cursor-not-allowed"
          onClick={() => toast.error("Screen is locked")}
        >
          <div className="text-center">
            {/* <div className="text-6xl mb-4 text-white opacity-80"><FaLock /></div> */}
            <div className="bg-white/10 text-white px-8 py-3 rounded-full font-bold backdrop-blur-xl border border-white/20 shadow-2xl">
              Console Locked — Press Esc to Unlock
            </div>
          </div>
          
          <button 
            className="absolute top-6 right-6 z-[10001] bg-white/10 hover:bg-white/20 text-white p-3 rounded-full pointer-events-auto transition-all"
            onClick={(e) => { e.stopPropagation(); setIsLocked(false); }}
          >
            {/* <FaUnlock /> */}
          </button>
        </div>
      )}
    </div>
  );
};

export default Dashboard;