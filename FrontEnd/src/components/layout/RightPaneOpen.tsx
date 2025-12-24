import React from "react";
// import MapsView from "../create/Maps.tsx";
// import ZonesPanel from "../create/zones.tsx";
// import AnalyticsView from "../../features/analytics/AnalyticsView";
import "../test/RightPane";
type MapsData = {
  list: any[];
  setList: React.Dispatch<React.SetStateAction<any[]>>;
  selected: any;
  setSelected: React.Dispatch<React.SetStateAction<any>>;
  requestV1: (path: string, options?: RequestInit) => Promise<any>;
};

type StatsData = Record<string, unknown>;

type RightPaneProps = {
  page: string;
  onClose: () => void;
  mapsData: MapsData;
  statsData: StatsData;
};

const RightPane: React.FC<RightPaneProps> = ({ page, onClose, mapsData, statsData }) => {
  const renderContent = () => {
    switch (page) {
      case "maps":
        return 
        // <MapsView {...mapsData} />;
      case "zones":
        return <>
          {/* <ZonesPanel/> */}
          <div className="p-6 text-slate-500">Zones View (Under Construction)</div>
        </>;
      case "waypoints":
        return <div className="p-6 text-slate-500">Waypoints View (Under Construction)</div>;
      case "missions":
        return <div className="p-6 text-slate-500">Missions View (Under Construction)</div>;
      case "users":
        return <div className="p-6 text-slate-500">Users View (Under Construction)</div>;
      case "analytics":
      case "stats":
        // return <AnalyticsView stats={statsData} />;
      case "chat":
        return <div className="p-6 text-slate-500">Chat Interface (Under Construction)</div>;
      default:
        return <div className="p-6 text-slate-500">Settings/Other for {page}</div>;
    }
  };

  const getTitle = () => page.charAt(0).toUpperCase() + page.slice(1).replace("-", " ");

  return (
    <aside className="z-30 flex h-full flex-1 max-w-[720px] flex-col border-l border-slate-200 bg-white shadow-2xl transition-all duration-300 ease-in-out">
      <div className="flex h-14 shrink-0 items-center justify-between border-b border-slate-100 bg-slate-50/50 px-6">
        <h2 className="text-lg font-bold text-slate-800">{getTitle()}</h2>
        <button
          onClick={onClose}
          className="flex h-8 w-8 items-center justify-center rounded-full text-slate-400 transition-colors hover:bg-slate-200 hover:text-slate-700"
          aria-label="Close right pane"
        >
          ✕
        </button>
      </div>

      <div className="flex-1 overflow-y-auto">{renderContent()}</div>
    </aside>
  );
};

export default RightPane;
