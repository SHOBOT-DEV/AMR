// new: right-side page (renders when a widen icon is clicked)
  const [rightPage, setRightPage] = useState(null);

    // Add zone search state
  const [zoneSearchField, setZoneSearchField] = useState("any");
  const [zoneSearchTerm, setZoneSearchTerm] = useState("");

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

    // UI
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


