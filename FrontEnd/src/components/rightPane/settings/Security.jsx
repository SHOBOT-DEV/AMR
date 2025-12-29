//logic
const persistSecurityPref = useCallback(
    (key, value, previousValue) => {
      requestV1("/settings/security", {
        method: "PATCH",
        body: JSON.stringify({ [key]: value }),
      }).catch((error) => {
        console.error("Security pref update failed", error);
        toast.error(error.message || "Failed to update security preference");
        setSecurityPreferences((prev) => ({ ...prev, [key]: previousValue }));
      });
    },
    [requestV1],
  );

  const toggleSecurityPref = (key) => {
    setSecurityPreferences((prev) => {
      const nextValue = !prev[key];
      persistSecurityPref(key, nextValue, prev[key]);
      return {
        ...prev,
        [key]: nextValue,
      };
    });
  };

    if (securityRes.data) {
          setSecurityPreferences((prev) => ({ ...prev, ...securityRes.data }));
        }
        if (Array.isArray(securityEventsRes.items)) {
          setSecurityEvents(securityEventsRes.items);
        }


          securityPreferences={securityPreferences}
            toggleSecurityPref={toggleSecurityPref}
            securityEvents={securityEvents}

            const [securityPreferences, setSecurityPreferences] = useState({
    twoFactor: true,
    autoLock: true,
    anomalyAlerts: true,
  });

  const [securityEvents, setSecurityEvents] = useState([
    {
      id: "sec1",
      ts: "09:44",
      actor: "ops-admin",
      action: "API token created",
      context: "Main console",
    },
    {
      id: "sec2",
      ts: "08:12",
      actor: "robot-01",
      action: "Cert renewed",
      context: "Device",
    },
  ]);


// UI

        {/* SECURITY */}
        {rightPage === "security" && (
          <div className="security-pane">
            <div className="security-toggles">
              {Object.entries(securityPreferences).map(([key, value]) => (
                <label key={key} className="toggle-row">
                  <input type="checkbox" checked={value} onChange={() => toggleSecurityPref(key)} />
                  <div>
                    <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                    <p>{value ? "Enabled" : "Disabled"}</p>
                  </div>
                </label>
              ))}
            </div>
            <table className="security-table">
              <thead>
                <tr>
                  <th>Time</th>
                  <th>Actor</th>
                  <th>Action</th>
                  <th>Context</th>
                </tr>
              </thead>
              <tbody>
                {securityEvents.map((evt) => (
                  <tr key={evt.id}>
                    <td>{evt.ts}</td>
                    <td>{evt.actor}</td>
                    <td>{evt.action}</td>
                    <td>{evt.context}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}