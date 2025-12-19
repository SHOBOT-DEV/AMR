import React from "react";
// Import your view components here (ensure the paths are correct based on your folder structure)
import MapsView from "../../features/maps/MapsView";
import AnalyticsView from "../../features/analytics/AnalyticsView"; 
// import ZonesView from "../../features/zones/ZonesView"; // Example

const RightPane = ({ page, onClose, mapsData, statsData }) => {
  
  const renderContent = () => {
    switch (page) {
      case "maps":
        return <MapsView {...mapsData} />;
      case "zones":
        return <div className="p-6 text-slate-500">Zones View (Under Construction)</div>;
      case "waypoints":
        return <div className="p-6 text-slate-500">Waypoints View (Under Construction)</div>;
      case "missions":
        return <div className="p-6 text-slate-500">Missions View (Under Construction)</div>;
      case "users":
        return <div className="p-6 text-slate-500">Users View (Under Construction)</div>;
      case "analytics":
        return <AnalyticsView stats={statsData} />;
      case "stats":
        return <AnalyticsView stats={statsData} />;
      case "chat":
        return <div className="p-6 text-slate-500">Chat Interface (Under Construction)</div>;
      default:
        return <div className="p-6 text-slate-500">Settings/Other for {page}</div>;
    }
  };

  const getTitle = () => {
    return page.charAt(0).toUpperCase() + page.slice(1).replace("-", " ");
  };

  return (
    <aside className="w-[400px] xl:w-[500px] h-full bg-white border-l border-slate-200 flex flex-col shadow-2xl z-30 transition-all duration-300 ease-in-out">
      {/* Pane Header */}
      <div className="h-14 flex items-center justify-between px-6 border-b border-slate-100 shrink-0 bg-slate-50/50">
        <h2 className="text-lg font-bold text-slate-800">{getTitle()}</h2>
        <button 
          onClick={onClose}
          className="w-8 h-8 flex items-center justify-center rounded-full hover:bg-slate-200 text-slate-400 hover:text-slate-700 transition-colors"
        >
          ✕
        </button>
      </div>

      {/* Pane Body */}
      <div className="flex-1 overflow-y-auto">
        {renderContent()}
      </div>
    </aside>
  );
};

export default RightPane;