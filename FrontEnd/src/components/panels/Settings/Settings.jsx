// RightPane logic
  const {
    rightPage,
    setRightPage,
    // settings
    accountProfile,
    handleAccountChange,
    profileError,
    profileSuccess,
    profileSaving,
    handleSaveProfile,
    selectedTheme,
    setSelectedTheme,
    securityPreferences,
    toggleSecurityPref,
    securityEvents,
    integrationItems,
    toggleIntegrationStatus,
  } = props;

// UI
        {/* ROBOT SETTINGS */}
        {rightPage === "robot-settings" && (
          <div className="settings-pane">
            {Object.entries(robotSettingsState).map(([key, value]) => (
              <label key={key} className="toggle-row">
                <input type="checkbox" checked={value} onChange={() => toggleRobotSetting(key)} />
                <div>
                  <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                  <p>{value ? "Enabled" : "Disabled"}</p>
                </div>
              </label>
            ))}
          </div>
        )}
