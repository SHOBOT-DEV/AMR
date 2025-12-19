import React, { useState } from "react";
import { FaBatteryHalf, FaPlay, FaPause, FaExpand, FaCompress, FaLock, FaUnlock, FaSignOutAlt } from "react-icons/fa";
import { toast } from "react-hot-toast";

const Header = ({ 
  bridgeStatus, emergencyClicked, setEmergencyClicked, handleLogout, 
  isLocked, setIsLocked, batteryLevel, setBatteryModalOpen, layoutRef 
}) => {
  const [isPlaying, setIsPlaying] = useState(false);
  const [isFullScreen, setIsFullScreen] = useState(false);
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);

  const toggleFullScreen = async () => {
    if (!document.fullscreenElement) {
      await layoutRef.current?.requestFullscreen();
      setIsFullScreen(true);
    } else {
      await document.exitFullscreen();
      setIsFullScreen(false);
    }
  };

  const getBatteryColor = (level) => {
    if (level < 20) return "#ef4444";
    if (level < 50) return "#f59e0b";
    return "#10b981";
  };

  return (
    <header className="fixed top-0 left-0 right-0 h-14 bg-white border-b border-slate-200 flex items-center justify-between px-4 z-50 shadow-sm">
      {/* Left Group */}
      <div className="flex items-center gap-3 w-48"> {/* Fixed width to match sidebar alignment roughly */}
        <div className={`flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border ${bridgeStatus.connected ? "bg-emerald-50 text-emerald-700 border-emerald-200" : "bg-red-50 text-red-700 border-red-200"}`}>
          <span className={`w-2 h-2 rounded-full ${bridgeStatus.connected ? "bg-emerald-500" : "bg-red-500 animate-pulse"}`} />
          <div className="flex flex-col leading-none">
            <span className="font-bold">AMR Bridge</span>
            <span className="opacity-75 text-[9px] uppercase tracking-wider">{bridgeStatus.connected ? "Online" : "Offline"}</span>
          </div>
        </div>
      </div>

      {/* Center Group: Emergency Stop */}
      <div className="absolute left-1/2 -translate-x-1/2">
        <button
          onClick={() => setEmergencyClicked(!emergencyClicked)}
          className={`flex items-center gap-2 px-6 py-1.5 rounded-full font-bold transition-all duration-200 shadow-sm ${
            emergencyClicked 
              ? "bg-red-600 text-white shadow-red-200 scale-105" 
              : "bg-white text-red-600 border border-red-100 hover:bg-red-50"
          }`}
        >
          <span className={`w-3 h-3 rounded-full ${emergencyClicked ? "bg-white animate-ping" : "bg-red-600"}`} />
          Emergency Stop
        </button>
      </div>

      {/* Right Group */}
      <div className="flex items-center gap-3">
        <div className="relative">
          <div 
            className="flex items-center gap-2 bg-slate-100 hover:bg-slate-200 text-slate-700 px-3 py-1.5 rounded-lg cursor-pointer transition-colors"
            onClick={() => setIsDropdownOpen(!isDropdownOpen)}
          >
            <button 
              className={`p-1 rounded-md transition-colors ${isPlaying ? "bg-amber-100 text-amber-600" : "bg-emerald-100 text-emerald-600"}`}
              onClick={(e) => { e.stopPropagation(); setIsPlaying(!isPlaying); }}
            >
              {isPlaying ? <FaPause size={10} /> : <FaPlay size={10} />}
            </button>
            <span className="text-xs font-bold">Mission Status</span>
          </div>
          {isDropdownOpen && (
            <div className="absolute top-full right-0 mt-2 w-56 bg-white border border-slate-200 rounded-lg shadow-xl p-3 z-50">
              <p className="text-[10px] text-slate-400 uppercase font-bold mb-1">Current Mission</p>
              <p className="font-medium text-sm text-slate-800">Idle - No active mission</p>
            </div>
          )}
        </div>

        <button 
          onClick={() => setBatteryModalOpen(true)}
          className="flex items-center gap-2 px-3 py-1.5 rounded-lg hover:bg-slate-100 transition-colors border border-transparent hover:border-slate-200"
        >
          <FaBatteryHalf color={getBatteryColor(batteryLevel)} size={16} />
          <span className="text-xs font-bold text-slate-700">{batteryLevel}%</span>
        </button>

        <div className="h-6 w-px bg-slate-200 mx-1" />
        
        <button onClick={toggleFullScreen} className="p-2 text-slate-400 hover:text-slate-800 transition-colors">
          {isFullScreen ? <FaCompress /> : <FaExpand />}
        </button>
        
        <button 
          onClick={() => { setIsLocked(!isLocked); toast.success(isLocked ? "Unlocked" : "Locked"); }} 
          className={`p-2 transition-colors ${isLocked ? "text-red-500" : "text-slate-400 hover:text-slate-800"}`}
        >
          {isLocked ? <FaLock /> : <FaUnlock />}
        </button>

        <button onClick={handleLogout} className="p-2 text-slate-400 hover:text-red-600 transition-colors">
          <FaSignOutAlt />
        </button>
      </div>
    </header>
  );
};

export default Header;