import React, { useState, useRef } from "react";
import "./RightPane.css";
import {
  FaPlus,
  FaTrash,
  FaMicrophone,
  FaPaperPlane,
} from "react-icons/fa";

/*
Props expected (pass these from MainPage.jsx):
{
  rightPage, setRightPage,
  // maps
  mapsList, selectedMap, setSelectedMap, mapSearchField, setMapSearchField,
  mapSearchTerm, setMapSearchTerm, createNewMapImmediate, handleActivateMap,
  handleMapAction,
  // zones
  zones, zoneFormOpen, setZoneFormOpen, zoneForm, setZoneForm, setZones,
  zoneSearchField, setZoneSearchField, zoneSearchTerm, setZoneSearchTerm,
  // waypoints
  waypoints, waypointFormOpen, setWaypointFormOpen, waypointForm, setWaypointForm,
  handleSelectWaypoint, setSelectedWaypointId,
  // users
  users, usersLoading, usersError, loadUsers, selectedUserId, setSelectedUserId,
  userActionLoading, handleResetUserPassword,
  // missions
  missions, missionFormOpen, setMissionFormOpen, missionForm, setMissionForm,
  selectedMissionId, setSelectedMissionId, handleSelectMission, setMissions,
  // analytics / diagnostics / logs / mission-history / bags
  analyticsSummary, analyticsSeries, analyticsAlerts, diagnosticsPanels,
  logEvents, missionHistory, bagFiles,
  // robot settings, account, appearance, security, integrations
  robotSettingsState, toggleRobotSetting,
  accountProfile, handleAccountChange, profileError, profileSuccess, profileSaving, handleSaveProfile,
  selectedTheme, setSelectedTheme,
  securityPreferences, toggleSecurityPref, securityEvents,
  integrationItems, toggleIntegrationStatus,
  // stats
  overview, missionTrend, monthlyMovement, batterySeries, batteryStatus, turns,
  statsLoading, statsError, lineChartSize, buildLinePath, buildSimplePath, analyticsChartSize, analyticsPath,
  // chat
  chatMessages, chatInput, setChatInput, handleSendMessage, handleKeyPress,
  isRecording, handleMicClick, isTyping, handleSuggestionClick, chatQuickPrompts, chatContainerRef, formatTimestamp, latestMessage
}
*/

  // Render function: reproduces the same right-pane layout previously inline
  return (
    <aside className="right-pane" role="region" aria-label="Right pane">
      <div className="right-pane-header">
        <strong style={{ fontSize: "1.8rem" }}>
          {rightPage ? rightPage.charAt(0).toUpperCase() + rightPage.slice(1) : ""}
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

        {/* fallback */}
        {![
          "maps","zones","waypoints","missions","users","analytics","diagnostics","logs","mission-logs","robot-bags","robot-settings","account","appearance","security","integrations","chat","stats"
        ].includes(rightPage) && <div>{rightPage}</div>}
      </div>
    </aside>
  );
};

export default RightPane;
