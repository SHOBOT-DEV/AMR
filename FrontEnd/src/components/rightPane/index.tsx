import React from "react";
import { FaTimes } from "react-icons/fa";
import MapsPane from "./MapsPane";
import ZonesPane from "./ZonesPane";
import UsersPane from "./UsersPane";
import ChatPane from "./ChatPane";
import AnalyticsPane from "./AnalyticsPane";
import SettingsPane from "./SettingsPane";
import Stats from "./Stats"; // Your existing component

const RightPane = (props: any) => {
  const { rightPage, setRightPage } = props;

  console.log(rightPage);
  return (
    <aside
      className="fixed right-0 top-14 w-full lg:w-1/2 h-[calc(100vh-3.5rem)] bg-white shadow-2xl z-40 flex flex-col border-l border-slate-200 text-slate-900 transition-all duration-300"
      aria-label="Right pane"
    >
      {/* Header */}
      <div className="flex items-center justify-between px-6 py-4 border-b border-slate-100 bg-gradient-to-b from-sky-50/50 to-white">
        <strong className="text-xl font-bold text-slate-800 capitalize">
          {rightPage?.replace("-", " ")}
        </strong>
        <button
          className="p-2 rounded-full text-slate-400 hover:text-slate-600 hover:bg-slate-100 transition-colors"
          onClick={() => setRightPage(null)}
          aria-label="Close"
        >
          <FaTimes />
        </button>
      </div>

      {/* Body Content Switcher */}
      <div className="flex-1 overflow-y-auto p-4 bg-white">
        {rightPage === "maps" && <MapsPane {...props} />}
        
        {(rightPage === "zones" || rightPage === "waypoints" || rightPage === "missions") && (
          <ZonesPane {...props} type={rightPage} />
        )}
        
        {rightPage === "users" && <UsersPane {...props} />}
        
        {(rightPage === "analytics" || rightPage === "logs" || rightPage === "robot-bags") && (
          <AnalyticsPane {...props} view={rightPage} />
        )}
        
        {rightPage === "chat" && <ChatPane {...props} />}
        
        {rightPage === "stats" && <Stats {...props} />}
        
        {["robot-settings", "security", "integrations", "account", "appearance"].includes(rightPage) && (
          <SettingsPane {...props} category={rightPage} />
        )}
      </div>
    </aside>
  );
};

export default RightPane;